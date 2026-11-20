//! Integer types for the dispatcher's internal point and entry handles.

use core::fmt::Debug;
use core::hash::Hash;

/// Seals [`Index`] so that only the integer types below implement it.
mod sealed {
    /// Implemented exactly for the types that implement [`super::Index`].
    pub trait Sealed {}
}

/// Unsigned integer type a [`Dispatcher`](super::Dispatcher) stores point and entry handles
/// in: `u16`, `u32` or `usize` (the default).
///
/// Public handles stay [`TimePoint`](crate::TimePoint)`(usize)`; the index type only sets the
/// width of the dispatcher's internal arrays (adjacency targets, range offsets, precedence
/// counters, activation links). A network fits iff its point count and its entry count
/// (`2 * edges + waits`) are both at most the type's maximum value, so that every index and
/// every range offset, including the one-past-the-end offset, is representable. Loading a
/// network that does not fit fails with
/// [`DispatchError::IndexTypeTooSmall`](super::DispatchError::IndexTypeTooSmall). Choosing
/// capacities `N, E <= MAX` rules that out statically.
///
/// # Memory
///
/// Measured `size_of` per point and per entry capacity slot:
///
/// | Target | `usize` | `u32` | `u16` |
/// | --- | --- | --- | --- |
/// | `thumbv7em-none-eabihf` (32-bit) | 80 + 16 | 80 + 16 | 76 + 16 |
/// | `x86_64` | 96 + 24 | 80 + 16 | 76 + 16 |
///
/// Entries and windows hold `i64` ticks and are 8-byte aligned, so on 32-bit targets a narrow
/// index saves only the per-point offsets and counters.
///
/// The trait is sealed: it cannot be implemented outside this crate.
pub trait Index: sealed::Sealed + Copy + Eq + Ord + Hash + Debug + 'static {
    /// The value zero, for `const` initialisers.
    const ZERO: Self;

    /// Converts `value`, or `None` if it does not fit.
    fn from_usize(value: usize) -> Option<Self>;

    /// Widens to `usize`.
    ///
    /// Exact for every value obtained from [`Index::from_usize`]; on targets where `usize` is
    /// narrower than `Self` other values map to `usize::MAX`.
    fn to_usize(self) -> usize;

    /// Subtracts one, staying at zero.
    #[must_use]
    fn saturating_dec(self) -> Self;
}

/// Implements [`Index`] for unsigned integer types.
macro_rules! impl_index {
    ($($t:ty),*) => {$(
        impl sealed::Sealed for $t {}

        impl Index for $t {
            const ZERO: Self = 0;

            fn from_usize(value: usize) -> Option<Self> {
                Self::try_from(value).ok()
            }

            fn to_usize(self) -> usize {
                usize::try_from(self).unwrap_or(usize::MAX)
            }

            fn saturating_dec(self) -> Self {
                self.saturating_sub(1)
            }
        }
    )*};
}

impl_index!(u16, u32, usize);
