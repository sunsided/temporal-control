//! Integer time ticks.

/// A point or duration in time, in integer ticks.
///
/// Integer ticks keep arithmetic deterministic across targets. The tick length (for example one
/// millisecond) is fixed by the integrator, not by this crate. All arithmetic on ticks inside
/// this crate is checked; overflow is reported as [`Error::Overflow`](crate::Error::Overflow).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Ticks(pub i64);

impl Ticks {
    /// Zero ticks.
    pub const ZERO: Self = Self(0);

    /// Adds two tick values, returning `None` on overflow.
    #[must_use]
    pub const fn checked_add(self, rhs: Self) -> Option<Self> {
        match self.0.checked_add(rhs.0) {
            Some(v) => Some(Self(v)),
            None => None,
        }
    }

    /// Subtracts `rhs`, returning `None` on overflow.
    #[must_use]
    pub const fn checked_sub(self, rhs: Self) -> Option<Self> {
        match self.0.checked_sub(rhs.0) {
            Some(v) => Some(Self(v)),
            None => None,
        }
    }

    /// Negates a tick value, returning `None` for `i64::MIN`.
    #[must_use]
    pub const fn checked_neg(self) -> Option<Self> {
        match self.0.checked_neg() {
            Some(v) => Some(Self(v)),
            None => None,
        }
    }
}
