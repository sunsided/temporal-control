//! Minimal (all-pairs tightest) networks.

use alloc::vec::Vec;

use super::TimePoint;
use crate::{Error, Ticks};

/// The minimal network of a consistent STN.
///
/// For every ordered pair `(i, j)`, [`Self::upper`] is the tightest upper bound on `t_j - t_i`
/// implied by the constraints. Every bound is tight: some schedule attains it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MinimalNetwork {
    /// Number of time points.
    pub(super) n: usize,
    /// Row-major `n * n` shortest-path distances; `None` means unbounded. No entry is
    /// `i64::MIN`, so every entry can be negated.
    pub(super) dist: Vec<Option<Ticks>>,
}

impl MinimalNetwork {
    /// Wraps a distance matrix, ensuring every entry can be negated.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if an entry is `i64::MIN`.
    pub(crate) fn new(n: usize, dist: Vec<Option<Ticks>>) -> Result<Self, Error> {
        if dist.iter().flatten().any(|d| d.0 == i64::MIN) {
            return Err(Error::Overflow);
        }

        Ok(Self { n, dist })
    }

    /// Number of time points.
    #[must_use]
    pub fn time_point_count(&self) -> usize {
        self.n
    }

    /// Tightest upper bound on `t_to - t_from`; `None` if unbounded or a handle is unknown.
    #[must_use]
    pub fn upper(&self, from: TimePoint, to: TimePoint) -> Option<Ticks> {
        if from.0 >= self.n || to.0 >= self.n {
            return None;
        }

        self.dist[from.0 * self.n + to.0]
    }

    /// Tightest lower bound on `t_to - t_from`; `None` if unbounded or a handle is unknown.
    #[must_use]
    pub fn lower(&self, from: TimePoint, to: TimePoint) -> Option<Ticks> {
        self.upper(to, from).and_then(Ticks::checked_neg)
    }
}
