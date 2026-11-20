//! CRC-32/ISO-HDLC, table-driven, with the table built at compile time.

/// Reflected generator polynomial `0x04C11DB7` of CRC-32/ISO-HDLC.
const POLYNOMIAL: u32 = 0xEDB8_8320;

/// Remainder of every byte value, for byte-at-a-time processing.
const TABLE: [u32; 256] = table();

/// Builds [`TABLE`].
const fn table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut byte = 0u32;

    while byte < 256 {
        let mut crc = byte;
        let mut bit = 0;

        while bit < 8 {
            crc = if crc & 1 == 0 {
                crc >> 1
            } else {
                (crc >> 1) ^ POLYNOMIAL
            };
            bit += 1;
        }

        table[byte as usize] = crc;
        byte += 1;
    }

    table
}

/// CRC-32/ISO-HDLC of `bytes`, the checksum of zlib, PNG and Ethernet.
///
/// Parameters: polynomial `0x04C11DB7` (reflected `0xEDB88320`), initial value and final XOR
/// `0xFFFFFFFF`, input and output reflected. Check value: `crc32(b"123456789") ==
/// 0xCBF4_3926`. Detects every error burst of up to 32 bits, in particular every single-bit
/// error.
///
/// # Examples
///
/// ```
/// use temporal_control::dispatch::encoding::crc32;
///
/// assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
/// assert_eq!(crc32(b""), 0);
/// ```
#[must_use]
pub fn crc32(bytes: &[u8]) -> u32 {
    !bytes.iter().fold(!0u32, |crc, &b| {
        TABLE[usize::from(crc.to_le_bytes()[0] ^ b)] ^ (crc >> 8)
    })
}
