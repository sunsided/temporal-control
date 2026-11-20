//! Compact, checksummed binary encoding of a dispatchable network.
//!
//! The ground side encodes a network once ([`encode_into`], or
//! [`DispatchableNetwork::encode`](crate::DispatchableNetwork::encode) with `alloc`); the
//! onboard side validates and loads it with
//! [`Dispatcher::load_encoded`](super::Dispatcher::load_encoded), which reads the records
//! directly from the received bytes, needs no `alloc` and never panics.
//!
//! # Format, version 1
//!
//! All integers are little-endian and fixed-width; there is no padding or alignment.
//!
//! | Offset | Size | Field |
//! | --- | --- | --- |
//! | 0 | 4 | magic `TCDN` (`54 43 44 4E`) |
//! | 4 | 2 | format version, `u16`, currently [`VERSION`] = 1 |
//! | 6 | 2 | reserved, `u16`, must be 0 |
//! | 8 | 4 | point count `n`, `u32` |
//! | 12 | 4 | edge count `e`, `u32` |
//! | 16 | 4 | wait count `w`, `u32` |
//! | 20 | `21 n` | point records |
//! | `20 + 21 n` | `16 e` | edge records |
//! | `20 + 21 n + 16 e` | `16 w` | wait records |
//! | end - 4 | 4 | CRC-32/ISO-HDLC ([`crc32()`]) of every byte before it, `u32` |
//!
//! Records (point indices are `u32`, ticks `i64`):
//!
//! - point: tag `u8` (0 executable, 1 contingent), activation `u32`, `lo` `i64`, `hi` `i64`;
//!   an executable point has all 20 bytes after the tag zero;
//! - edge `t_to - t_from <= weight`: `from` `u32`, `to` `u32`, `weight` `i64`;
//! - wait: `waiter` `u32`, `contingent` `u32`, `delay` `i64`.
//!
//! Fixed-width records give random access to record `i`, which the validator needs to
//! follow activation chains without copying the network. The total length is
//! `24 + 21 n + 16 (e + w)` bytes and must match the buffer exactly.
//!
//! # Decoding
//!
//! Checks, in order, each with its own [`DecodeError`]: at least a header and a checksum
//! ([`DecodeError::Truncated`]), magic, version, exact length from the counts, the CRC, the
//! reserved field and point tags (canonical form), then everything
//! [`Dispatcher::load`](super::Dispatcher::load) checks (indices, contingent bounds,
//! activation chains, waits, capacities, index type). A decoder for a later version is
//! expected to reject version 1 encodings it cannot read explicitly, never by checksum
//! failure: the magic and version come first so that the checksum location may change.
//!
//! # Examples
//!
//! ```
//! use temporal_control::dispatch::encoding::{self, encoded_len};
//! use temporal_control::{DispatchEdge, Dispatcher, PointKind, Ticks, TimePoint};
//!
//! let points = [PointKind::Executable; 2];
//! let edges = [DispatchEdge { from: TimePoint(0), to: TimePoint(1), weight: Ticks(10) }];
//! let mut buf = [0u8; 128];
//! let len = encoding::encode_into(&points, &edges, &[], &mut buf).expect("fits");
//! assert_eq!(Some(len), encoded_len(2, 1, 0));
//!
//! let mut d = Dispatcher::<2, 2>::EMPTY;
//! d.load_encoded(&buf[..len]).expect("valid");
//! ```

mod crc32;

use core::fmt;

use super::source::Source;
use super::{DispatchEdge, DispatchError, PointKind, Wait};
use crate::{Ticks, TimePoint};

pub use crc32::crc32;

/// First four bytes of every encoding.
pub const MAGIC: [u8; 4] = *b"TCDN";

/// Format version written by [`encode_into`] and accepted by the decoder.
pub const VERSION: u16 = 1;

/// Length of the fixed header in bytes.
const HEADER_LEN: usize = 20;

/// Length of the trailing checksum in bytes.
const CRC_LEN: usize = 4;

/// Length of a point record in bytes.
const POINT_LEN: usize = 21;

/// Length of an edge or wait record in bytes.
const PAIR_LEN: usize = 16;

/// Point record tag of an executable point.
const TAG_EXECUTABLE: u8 = 0;

/// Point record tag of a contingent point.
const TAG_CONTINGENT: u8 = 1;

/// Reasons an encoding was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum DecodeError {
    /// Shorter than a header plus a checksum.
    Truncated,
    /// The first four bytes are not [`MAGIC`].
    BadMagic,
    /// The version field is not [`VERSION`]; carries the version found.
    UnsupportedVersion(u16),
    /// The buffer length differs from the length the header's counts imply (truncated, or
    /// trailing bytes).
    LengthMismatch {
        /// Length implied by the counts, in bytes.
        expected: u64,
        /// Length of the buffer, in bytes.
        actual: usize,
    },
    /// The CRC-32 does not match the content.
    ChecksumMismatch,
    /// The reserved field is not zero, a point tag is unknown, or an executable point record
    /// has non-zero payload bytes.
    NonCanonical,
    /// The records decode, but the network is rejected as by
    /// [`Dispatcher::load`](super::Dispatcher::load).
    Network(DispatchError),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated => f.write_str("encoding shorter than header and checksum"),
            Self::BadMagic => f.write_str("not a dispatchable network encoding (bad magic)"),
            Self::UnsupportedVersion(v) => write!(f, "unsupported encoding version {v}"),
            Self::LengthMismatch { expected, actual } => {
                write!(f, "encoding is {actual} bytes, header implies {expected}")
            }
            Self::ChecksumMismatch => f.write_str("encoding checksum mismatch"),
            Self::NonCanonical => f.write_str("encoding not in canonical form"),
            Self::Network(e) => write!(f, "encoded network rejected: {e}"),
        }
    }
}

impl core::error::Error for DecodeError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Network(e) => Some(e),
            _ => None,
        }
    }
}

/// Reasons a network could not be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum EncodeError {
    /// The output buffer is shorter than the encoding.
    BufferTooSmall {
        /// Bytes the encoding needs.
        needed: usize,
    },
    /// A count or a point index does not fit `u32`, or the length does not fit `usize`.
    TooLarge,
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooSmall { needed } => write!(f, "encoding needs {needed} bytes"),
            Self::TooLarge => f.write_str("network too large for the encoding"),
        }
    }
}

impl core::error::Error for EncodeError {}

/// Length in bytes of the encoding of a network with the given counts, or `None` if a count
/// exceeds `u32::MAX` or the length overflows `usize`.
#[must_use]
pub fn encoded_len(points: usize, edges: usize, waits: usize) -> Option<usize> {
    let counts = [points, edges, waits].map(|c| u32::try_from(c).ok());
    let [Some(n), Some(e), Some(w)] = counts else {
        return None;
    };

    usize::try_from(length_of(n, e, w)).ok()
}

/// Length of an encoding with these counts, computed in `u64` so it cannot overflow even
/// where `usize` has 32 bits.
fn length_of(points: u32, edges: u32, waits: u32) -> u64 {
    let fixed = (HEADER_LEN + CRC_LEN) as u64;
    let pairs = u64::from(edges) + u64::from(waits);

    fixed + POINT_LEN as u64 * u64::from(points) + PAIR_LEN as u64 * pairs
}

/// Encodes a network into `out` and returns the number of bytes written.
///
/// The network is not validated here; the decoder validates it. Bytes of `out` after the
/// returned length are left untouched.
///
/// # Errors
///
/// - [`EncodeError::TooLarge`] if a count or a point index exceeds `u32::MAX`.
/// - [`EncodeError::BufferTooSmall`] if `out` is shorter than [`encoded_len`].
pub fn encode_into(
    points: &[PointKind],
    edges: &[DispatchEdge],
    waits: &[Wait],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let len = encoded_len(points.len(), edges.len(), waits.len()).ok_or(EncodeError::TooLarge)?;
    let buf = out
        .get_mut(..len)
        .ok_or(EncodeError::BufferTooSmall { needed: len })?;
    let mut w = Writer { buf, at: 0 };

    w.header(points.len(), edges.len(), waits.len())?;

    for &p in points {
        w.point(p)?;
    }

    for e in edges {
        w.pair(e.from, e.to, e.weight)?;
    }

    for x in waits {
        w.pair(x.waiter, x.contingent, x.delay)?;
    }

    w.checksum()?;
    Ok(len)
}

/// Sequential writer into a buffer of exactly the encoding's length.
struct Writer<'a> {
    /// Destination, exactly [`encoded_len`] bytes.
    buf: &'a mut [u8],
    /// Bytes written so far.
    at: usize,
}

impl Writer<'_> {
    /// Appends `bytes`.
    ///
    /// # Errors
    ///
    /// [`EncodeError::BufferTooSmall`] if they do not fit (not reachable when the buffer has
    /// the computed length).
    fn put(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        let needed = self.buf.len();
        let end = self.at.saturating_add(bytes.len());
        let dst = self
            .buf
            .get_mut(self.at..end)
            .ok_or(EncodeError::BufferTooSmall { needed })?;
        dst.copy_from_slice(bytes);
        self.at = end;
        Ok(())
    }

    /// Appends a `u32` count or point index.
    ///
    /// # Errors
    ///
    /// [`EncodeError::TooLarge`] if `value` exceeds `u32::MAX`, or as for [`Self::put`].
    fn put_u32(&mut self, value: usize) -> Result<(), EncodeError> {
        let v = u32::try_from(value).map_err(|_| EncodeError::TooLarge)?;
        self.put(&v.to_le_bytes())
    }

    /// Appends a tick value.
    ///
    /// # Errors
    ///
    /// As for [`Self::put`].
    fn put_ticks(&mut self, t: Ticks) -> Result<(), EncodeError> {
        self.put(&t.0.to_le_bytes())
    }

    /// Writes the header.
    ///
    /// # Errors
    ///
    /// As for [`Self::put_u32`].
    fn header(&mut self, points: usize, edges: usize, waits: usize) -> Result<(), EncodeError> {
        self.put(&MAGIC)?;
        self.put(&VERSION.to_le_bytes())?;
        self.put(&0u16.to_le_bytes())?;
        self.put_u32(points)?;
        self.put_u32(edges)?;
        self.put_u32(waits)
    }

    /// Writes a point record.
    ///
    /// # Errors
    ///
    /// As for [`Self::put_u32`].
    fn point(&mut self, kind: PointKind) -> Result<(), EncodeError> {
        match kind {
            PointKind::Executable => self.put(&[0; POINT_LEN]),
            PointKind::Contingent { activation, lo, hi } => {
                self.put(&[TAG_CONTINGENT])?;
                self.put_u32(activation.0)?;
                self.put_ticks(lo)?;
                self.put_ticks(hi)
            }
        }
    }

    /// Writes an edge or wait record.
    ///
    /// # Errors
    ///
    /// As for [`Self::put_u32`].
    fn pair(&mut self, a: TimePoint, b: TimePoint, t: Ticks) -> Result<(), EncodeError> {
        self.put_u32(a.0)?;
        self.put_u32(b.0)?;
        self.put_ticks(t)
    }

    /// Writes the CRC of everything written so far.
    ///
    /// # Errors
    ///
    /// As for [`Self::put`].
    fn checksum(&mut self) -> Result<(), EncodeError> {
        let crc = crc32(self.buf.get(..self.at).unwrap_or_default());
        self.put(&crc.to_le_bytes())
    }
}

/// An encoding whose header, length, checksum and record tags have been checked.
///
/// Records are read on demand from the borrowed bytes through [`Source`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct Encoded<'a> {
    /// The whole encoding, checksum included.
    bytes: &'a [u8],
    /// Number of point records.
    points: usize,
    /// Number of edge records.
    edges: usize,
    /// Number of wait records.
    waits: usize,
}

impl<'a> Encoded<'a> {
    /// Checks the framing of `bytes`: everything except the network's structure.
    ///
    /// # Errors
    ///
    /// Every [`DecodeError`] except [`DecodeError::Network`], in the order of the module docs.
    pub(crate) fn parse(bytes: &'a [u8]) -> Result<Self, DecodeError> {
        if bytes.len() < HEADER_LEN + CRC_LEN {
            return Err(DecodeError::Truncated);
        }

        if bytes.get(..4) != Some(&MAGIC[..]) {
            return Err(DecodeError::BadMagic);
        }

        let version = read_u16(bytes, 4).ok_or(DecodeError::Truncated)?;

        if version != VERSION {
            return Err(DecodeError::UnsupportedVersion(version));
        }

        let encoded = Self::framed(bytes)?;
        encoded.verify_checksum()?;
        encoded.verify_canonical()?;
        Ok(encoded)
    }

    /// Reads the counts and checks that they match the buffer length exactly.
    ///
    /// # Errors
    ///
    /// [`DecodeError::LengthMismatch`].
    fn framed(bytes: &'a [u8]) -> Result<Self, DecodeError> {
        let count = |at| read_u32(bytes, at).ok_or(DecodeError::Truncated);
        let (n, e, w) = (count(8)?, count(12)?, count(16)?);
        let expected = length_of(n, e, w);

        if u64::try_from(bytes.len()).ok() != Some(expected) {
            return Err(DecodeError::LengthMismatch {
                expected,
                actual: bytes.len(),
            });
        }

        let widen = |c: u32| usize::try_from(c).map_err(|_| DecodeError::Truncated);

        Ok(Self {
            bytes,
            points: widen(n)?,
            edges: widen(e)?,
            waits: widen(w)?,
        })
    }

    /// Compares the trailing CRC with the CRC of the content.
    ///
    /// # Errors
    ///
    /// [`DecodeError::ChecksumMismatch`].
    fn verify_checksum(&self) -> Result<(), DecodeError> {
        let body_len = self.bytes.len().saturating_sub(CRC_LEN);
        let (body, stored) = self.bytes.split_at(body_len);
        let stored = stored.try_into().ok().map(u32::from_le_bytes);

        if stored != Some(crc32(body)) {
            return Err(DecodeError::ChecksumMismatch);
        }

        Ok(())
    }

    /// Checks the reserved field and every point record's tag and padding.
    ///
    /// # Errors
    ///
    /// [`DecodeError::NonCanonical`].
    fn verify_canonical(&self) -> Result<(), DecodeError> {
        let reserved_zero = read_u16(self.bytes, 6) == Some(0);
        let records_ok = (0..self.points).all(|i| {
            self.point_record(i).is_some_and(|r| match r[0] {
                TAG_EXECUTABLE => r[1..].iter().all(|&b| b == 0),
                TAG_CONTINGENT => true,
                _ => false,
            })
        });

        if !(reserved_zero && records_ok) {
            return Err(DecodeError::NonCanonical);
        }

        Ok(())
    }

    /// Raw point record `i`.
    fn point_record(&self, i: usize) -> Option<[u8; POINT_LEN]> {
        if i >= self.points {
            return None;
        }

        let at = HEADER_LEN.checked_add(i.checked_mul(POINT_LEN)?)?;
        read(self.bytes, at)
    }

    /// Edge or wait record `i` of the section starting after `skip` pair records.
    fn pair_record(&self, skip: usize, i: usize) -> Option<(TimePoint, TimePoint, Ticks)> {
        let points_len = self.points.checked_mul(POINT_LEN)?;
        let index = skip.checked_add(i)?.checked_mul(PAIR_LEN)?;
        let at = HEADER_LEN.checked_add(points_len)?.checked_add(index)?;
        let r: [u8; PAIR_LEN] = read(self.bytes, at)?;

        Some((point_at(&r, 0)?, point_at(&r, 4)?, ticks_at(&r, 8)?))
    }
}

impl Source for Encoded<'_> {
    fn point_count(&self) -> usize {
        self.points
    }

    fn edge_count(&self) -> usize {
        self.edges
    }

    fn wait_count(&self) -> usize {
        self.waits
    }

    fn point(&self, i: usize) -> Option<PointKind> {
        let r = self.point_record(i)?;

        match r[0] {
            TAG_EXECUTABLE => Some(PointKind::Executable),
            TAG_CONTINGENT => Some(PointKind::Contingent {
                activation: point_at(&r, 1)?,
                lo: ticks_at(&r, 5)?,
                hi: ticks_at(&r, 13)?,
            }),
            _ => None,
        }
    }

    fn edge(&self, i: usize) -> Option<DispatchEdge> {
        if i >= self.edges {
            return None;
        }

        let (from, to, weight) = self.pair_record(0, i)?;
        Some(DispatchEdge { from, to, weight })
    }

    fn wait(&self, i: usize) -> Option<Wait> {
        if i >= self.waits {
            return None;
        }

        let (waiter, contingent, delay) = self.pair_record(self.edges, i)?;
        Some(Wait {
            waiter,
            contingent,
            delay,
        })
    }
}

/// The `W` bytes at offset `at`, or `None` if out of range.
fn read<const W: usize>(bytes: &[u8], at: usize) -> Option<[u8; W]> {
    bytes.get(at..at.checked_add(W)?)?.try_into().ok()
}

/// Little-endian `u16` at offset `at`.
fn read_u16(bytes: &[u8], at: usize) -> Option<u16> {
    read(bytes, at).map(u16::from_le_bytes)
}

/// Little-endian `u32` at offset `at`.
fn read_u32(bytes: &[u8], at: usize) -> Option<u32> {
    read(bytes, at).map(u32::from_le_bytes)
}

/// Point index (`u32`) at offset `at`. An index that does not fit `usize` (only possible
/// where `usize` has 16 bits) becomes `usize::MAX`, which validation rejects as out of range.
fn point_at(bytes: &[u8], at: usize) -> Option<TimePoint> {
    read_u32(bytes, at).map(|v| TimePoint(usize::try_from(v).unwrap_or(usize::MAX)))
}

/// Tick value (`i64`) at offset `at`.
fn ticks_at(bytes: &[u8], at: usize) -> Option<Ticks> {
    read(bytes, at).map(|b| Ticks(i64::from_le_bytes(b)))
}

#[cfg(test)]
mod tests {
    //! Unit tests for the CRC and the framing checks.

    use super::*;

    /// The standard check value of CRC-32/ISO-HDLC.
    #[test]
    fn crc_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
        assert_eq!(crc32(b"a"), 0xE8B7_BE43);
    }

    /// The length formula matches the documented layout.
    #[test]
    fn length_formula() {
        assert_eq!(encoded_len(0, 0, 0), Some(24));
        assert_eq!(encoded_len(2, 3, 1), Some(24 + 42 + 64));
        assert_eq!(
            length_of(u32::MAX, u32::MAX, u32::MAX),
            24 + 53 * u64::from(u32::MAX)
        );
    }
}
