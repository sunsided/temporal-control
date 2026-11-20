//! Incremental consistency: keep the all-pairs distance matrix closed while constraints arrive.
//!
//! Adding the distance-graph edge `u -> v` with weight `w` to a consistent network with closed
//! distance matrix `D` creates a negative cycle iff `D[v][u] + w < 0`. Otherwise the new closed
//! matrix is `D'[a][b] = min(D[a][b], D[a][u] + w + D[v][b])`. Row `u` and column `v` do not
//! change under this update (that would need `D[v][u] + w < 0`), so it can run in place.
//!
//! A constraint has up to two edges. If both are added, the second sees `D'[u][v] =
//! min(D[u][v], hi)`, so the pair is rejected iff `D[v][u] + hi < 0`, `D[u][v] - lo < 0` or
//! `hi - lo < 0`. All three are checked before anything changes, so a rejected constraint leaves
//! the network untouched. The update itself runs on a copy of the matrix that replaces the
//! original only on success, so overflow or budget exhaustion leave it untouched as well.

use alloc::vec::Vec;

use super::{ConstraintId, MinimalNetwork, Stn, TimePoint, Verdict, bellman_ford, steps};
use crate::{Budget, Error, Ticks};

/// An STN that keeps its minimal network up to date as constraints are added.
///
/// Each [`IncrementalStn::add_constraint`] costs `O(n^2)` steps instead of the `O(n^3)` of a
/// full [`Stn::minimal_network`] recomputation. A constraint that would make the network
/// inconsistent is rejected: the network stays as it was and the verdict explains why.
///
/// # Examples
///
/// ```
/// use temporal_control::{Budget, IncrementalStn, Ticks, Verdict};
///
/// let mut stn = IncrementalStn::new();
/// let budget = Budget::UNLIMITED;
/// let a = stn.add_time_point(&budget).expect("fits");
/// let b = stn.add_time_point(&budget).expect("fits");
///
/// let ok = stn.add_constraint(a, b, Some(Ticks(5)), Some(Ticks(15)), &budget).expect("ok");
/// assert!(matches!(ok, Verdict::Consistent(_)));
///
/// // 20 <= t_b - t_a contradicts t_b - t_a <= 15 and is rejected.
/// let bad = stn.add_constraint(a, b, Some(Ticks(20)), None, &budget).expect("ok");
/// assert!(matches!(bad, Verdict::Inconsistent(_)));
/// assert_eq!(stn.stn().constraints().len(), 1);
/// assert_eq!(stn.upper(a, b), Some(Ticks(15)));
/// ```
#[derive(Debug, Clone, Default)]
pub struct IncrementalStn {
    /// The accepted constraints. Always consistent.
    stn: Stn,
    /// Row-major `n * n` closed distance matrix of `stn`; `None` is `+infinity`.
    dist: Vec<Option<Ticks>>,
}

impl IncrementalStn {
    /// Creates an empty network.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds an unconstrained time point.
    ///
    /// Costs `O(n^2)` steps to grow the matrix.
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if `(n + 1)^2` exceeds [`Budget::max_cells`].
    /// - [`Error::OutOfMemory`] if the allocator refuses the new matrix.
    /// - [`Error::BudgetExhausted`] if the step budget cannot cover the copy.
    /// - [`Error::Overflow`] if the time point count overflows.
    pub fn add_time_point(&mut self, budget: &Budget) -> Result<TimePoint, Error> {
        let n = self.stn.time_points;
        let m = n.checked_add(1).ok_or(Error::Overflow)?;
        let cells = m.checked_mul(m).ok_or(Error::MemoryBudgetExceeded)?;
        budget.check_cells(cells)?;
        budget.meter().spend(steps(cells)?)?;

        let mut dist = Vec::new();
        dist.try_reserve_exact(cells)
            .map_err(|_| Error::OutOfMemory)?;

        for row in self.dist.chunks(n.max(1)).take(n) {
            dist.extend_from_slice(row);
            dist.push(None);
        }

        dist.extend(core::iter::repeat_n(None, n));
        dist.push(Some(Ticks::ZERO));

        let tp = self.stn.add_time_point()?;
        self.dist = dist;
        Ok(tp)
    }

    /// Adds `lo <= t_to - t_from <= hi` if the network stays consistent.
    ///
    /// Returns the new [`ConstraintId`] on success. On inconsistency the constraint is not
    /// added and the verdict carries a negative cycle through the rejected constraint, whose id
    /// in the cycle is the one it would have received.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if `from` or `to` is not in this network.
    /// - [`Error::Overflow`] if `lo` is `i64::MIN` or a path length overflows. The network is
    ///   unchanged.
    /// - [`Error::BudgetExhausted`] if the `O(n^2)` update, or the explanation search after a
    ///   rejection, does not fit the step budget. The network is unchanged.
    /// - [`Error::Internal`] if the explanation search disagrees with the matrix (a bug).
    pub fn add_constraint(
        &mut self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
        budget: &Budget,
    ) -> Result<Verdict<ConstraintId>, Error> {
        let n = self.stn.time_points;

        if from.0 >= n || to.0 >= n {
            return Err(Error::UnknownTimePoint);
        }

        let neg_lo = lo
            .map(|l| l.checked_neg().ok_or(Error::Overflow))
            .transpose()?;
        let mut meter = budget.meter();

        if self.closes_negative_cycle(from.0, to.0, hi, neg_lo)? {
            return self.explain_rejection(from, to, lo, hi, budget);
        }

        let mut dist = copy_matrix(&self.dist)?;

        for (u, v, w) in [(from.0, to.0, hi), (to.0, from.0, neg_lo)] {
            if let Some(w) = w {
                meter.spend(steps(n * n)?)?;
                close_after_edge(&mut dist, n, u, v, w)?;
            }
        }

        let id = self.stn.add_constraint(from, to, lo, hi)?;
        self.dist = dist;
        Ok(Verdict::Consistent(id))
    }

    /// Tightest upper bound on `t_to - t_from`; `None` if unbounded or a handle is unknown.
    #[must_use]
    pub fn upper(&self, from: TimePoint, to: TimePoint) -> Option<Ticks> {
        let n = self.stn.time_points;

        if from.0 >= n || to.0 >= n {
            return None;
        }

        self.dist[from.0 * n + to.0]
    }

    /// Tightest lower bound on `t_to - t_from`; `None` if unbounded or a handle is unknown.
    #[must_use]
    pub fn lower(&self, from: TimePoint, to: TimePoint) -> Option<Ticks> {
        self.upper(to, from).and_then(Ticks::checked_neg)
    }

    /// The accepted constraints as a plain [`Stn`].
    #[must_use]
    pub fn stn(&self) -> &Stn {
        &self.stn
    }

    /// A snapshot of the current minimal network.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if an entry is `i64::MIN`; [`Error::OutOfMemory`] if the copy fails.
    pub fn minimal_network(&self) -> Result<MinimalNetwork, Error> {
        MinimalNetwork::new(self.stn.time_points, copy_matrix(&self.dist)?)
    }

    /// Whether adding the edges `u -> v` (`hi`) and `v -> u` (`neg_lo`) closes a negative cycle.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a cycle length overflows.
    fn closes_negative_cycle(
        &self,
        u: usize,
        v: usize,
        hi: Option<Ticks>,
        neg_lo: Option<Ticks>,
    ) -> Result<bool, Error> {
        let n = self.stn.time_points;
        let negative = |path: Option<Ticks>, edge: Option<Ticks>| -> Result<bool, Error> {
            match (path, edge) {
                (Some(dist), Some(weight)) => {
                    Ok(dist.checked_add(weight).ok_or(Error::Overflow)? < Ticks::ZERO)
                }
                _ => Ok(false),
            }
        };

        Ok(negative(self.dist[v * n + u], hi)?
            || negative(self.dist[u * n + v], neg_lo)?
            || negative(hi, neg_lo)?)
    }

    /// Builds the negative-cycle explanation for a rejected constraint.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::Overflow`], or [`Error::Internal`] if Bellman-Ford
    /// finds the extended network consistent.
    fn explain_rejection(
        &self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
        budget: &Budget,
    ) -> Result<Verdict<ConstraintId>, Error> {
        let mut extended = self.stn.clone();
        extended.add_constraint(from, to, lo, hi)?;

        match bellman_ford::run(&extended, &mut budget.meter())? {
            Verdict::Inconsistent(cycle) => Ok(Verdict::Inconsistent(cycle)),
            Verdict::Consistent(_) => Err(Error::Internal),
        }
    }
}

/// Copies a distance matrix with a fallible allocation.
///
/// # Errors
///
/// [`Error::OutOfMemory`] if the allocation fails.
fn copy_matrix(dist: &[Option<Ticks>]) -> Result<Vec<Option<Ticks>>, Error> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(dist.len())
        .map_err(|_| Error::OutOfMemory)?;
    copy.extend_from_slice(dist);
    Ok(copy)
}

/// Adds edge `u -> v` with weight `w` to the closed matrix `dist` and re-closes it in place.
///
/// The caller has ruled out a negative cycle through the edge, so row `u` and column `v` are
/// fixed points of the update and reading them while writing is sound.
///
/// # Errors
///
/// [`Error::Overflow`] if a candidate path length overflows; `dist` is then partially updated
/// and must be discarded.
fn close_after_edge(
    dist: &mut [Option<Ticks>],
    n: usize,
    u: usize,
    v: usize,
    w: Ticks,
) -> Result<(), Error> {
    for a in 0..n {
        let Some(d_au) = dist[a * n + u] else {
            continue;
        };

        let via = d_au.checked_add(w).ok_or(Error::Overflow)?;

        for b in 0..n {
            let Some(d_vb) = dist[v * n + b] else {
                continue;
            };

            let candidate = via.checked_add(d_vb).ok_or(Error::Overflow)?;
            let cell = &mut dist[a * n + b];

            if cell.is_none_or(|d| candidate < d) {
                *cell = Some(candidate);
            }
        }
    }

    Ok(())
}
