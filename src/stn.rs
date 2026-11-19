//! Simple Temporal Networks (STN).
//!
//! An STN is a set of time points and binary constraints `lo <= t_j - t_i <= hi`. Its distance
//! graph has an edge `i -> j` with weight `hi` and an edge `j -> i` with weight `-lo` per
//! constraint. The network is consistent iff the distance graph has no negative cycle
//! (Dechter, Meiri and Pearl 1991).
//!
//! ```mermaid
//! graph LR
//!     stn["Stn"] -->|"check_consistency (Bellman-Ford)"| v1["Verdict: Schedule or NegativeCycle"]
//!     stn -->|"minimal_network (Floyd-Warshall)"| v2["Verdict: MinimalNetwork or NegativeCycle"]
//! ```

mod bellman_ford;
mod explanation;
mod floyd_warshall;
mod minimal;

use alloc::vec::Vec;

use crate::{Budget, Error, Ticks};

pub use explanation::{Bound, CycleEdge, NegativeCycle};
pub use minimal::MinimalNetwork;

/// Handle of a time point inside one [`Stn`].
///
/// Handles are dense indices starting at zero, in creation order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TimePoint(pub usize);

/// Handle of a constraint inside one [`Stn`], in insertion order starting at zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ConstraintId(pub usize);

/// A binary constraint `lo <= t_to - t_from <= hi`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Constraint {
    /// The time point subtracted (`t_i`).
    pub from: TimePoint,
    /// The time point the difference is measured to (`t_j`).
    pub to: TimePoint,
    /// Lower bound on `t_to - t_from`; `None` means unbounded below.
    pub lo: Option<Ticks>,
    /// Upper bound on `t_to - t_from`; `None` means unbounded above.
    pub hi: Option<Ticks>,
}

/// Outcome of a check that answered the question.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict<T> {
    /// The network is consistent; the payload is a certificate or derived structure.
    Consistent(T),
    /// The network is inconsistent; the payload names the constraints forming a negative cycle.
    Inconsistent(NegativeCycle),
}

/// One assignment of a time to every time point that satisfies all constraints.
///
/// Serves as a checkable certificate of consistency.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Schedule {
    /// Time of each time point, indexed by [`TimePoint`] index.
    times: Vec<Ticks>,
}

impl Schedule {
    /// Time assigned to `tp`, or `None` if `tp` is not part of the network.
    #[must_use]
    pub fn time(&self, tp: TimePoint) -> Option<Ticks> {
        self.times.get(tp.0).copied()
    }

    /// All times, indexed by time point.
    #[must_use]
    pub fn times(&self) -> &[Ticks] {
        &self.times
    }
}

/// A directed, weighted distance-graph edge `from -> to` meaning `t_to - t_from <= weight`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Edge {
    /// Tail time point index.
    from: usize,
    /// Head time point index.
    to: usize,
    /// Edge weight, already negated for lower bounds.
    weight: Ticks,
    /// The constraint bound this edge encodes, for explanations.
    source: CycleEdge,
}

/// A Simple Temporal Network.
///
/// Construction is infallible apart from handle validation and the negation of lower bounds.
/// Algorithms are read-only and take an explicit [`Budget`].
#[derive(Debug, Clone, Default)]
pub struct Stn {
    /// Number of time points.
    time_points: usize,
    /// Constraints in insertion order; index is the [`ConstraintId`].
    constraints: Vec<Constraint>,
    /// Distance-graph edges derived from `constraints`, one per finite bound.
    edges: Vec<Edge>,
}

impl Stn {
    /// Creates an empty network.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a time point and returns its handle.
    pub fn add_time_point(&mut self) -> TimePoint {
        let tp = TimePoint(self.time_points);
        self.time_points += 1;
        tp
    }

    /// Number of time points.
    #[must_use]
    pub fn time_point_count(&self) -> usize {
        self.time_points
    }

    /// All constraints, indexed by [`ConstraintId`].
    #[must_use]
    pub fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    /// Looks up one constraint.
    #[must_use]
    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint> {
        self.constraints.get(id.0)
    }

    /// Adds the constraint `lo <= t_to - t_from <= hi`.
    ///
    /// Either bound may be `None` (unbounded). `lo > hi` is accepted and makes the network
    /// inconsistent, which the checks then explain.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if `from` or `to` is not in this network.
    /// - [`Error::Overflow`] if `lo` is `i64::MIN`, whose negation is not representable.
    pub fn add_constraint(
        &mut self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
    ) -> Result<ConstraintId, Error> {
        if from.0 >= self.time_points || to.0 >= self.time_points {
            return Err(Error::UnknownTimePoint);
        }

        let neg_lo = lo
            .map(|l| l.checked_neg().ok_or(Error::Overflow))
            .transpose()?;
        let id = ConstraintId(self.constraints.len());

        if let Some(w) = hi {
            self.push_edge(from, to, w, CycleEdge::new(id, Bound::Upper));
        }

        if let Some(w) = neg_lo {
            self.push_edge(to, from, w, CycleEdge::new(id, Bound::Lower));
        }

        self.constraints.push(Constraint { from, to, lo, hi });
        Ok(id)
    }

    /// Checks consistency with Bellman-Ford from a virtual source.
    ///
    /// Runs in `O(n * m)` steps and `O(n + m)` memory. A consistent verdict carries a
    /// [`Schedule`] that satisfies every constraint; an inconsistent verdict carries a
    /// [`NegativeCycle`].
    ///
    /// # Errors
    ///
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight overflows `i64`.
    /// - [`Error::Internal`] if cycle extraction breaks an invariant (a bug).
    pub fn check_consistency(&self, budget: &Budget) -> Result<Verdict<Schedule>, Error> {
        let mut meter = budget.meter();
        bellman_ford::run(self, &mut meter)
    }

    /// Computes the minimal network (all-pairs shortest paths) with Floyd-Warshall.
    ///
    /// Runs in `O(n^3)` steps and `n^2` cells. On inconsistency, the explanation is extracted
    /// with Bellman-Ford using the remaining budget; if Bellman-Ford disagrees, the result is
    /// [`Error::Internal`].
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if `n^2` exceeds [`Budget::max_cells`].
    /// - [`Error::OutOfMemory`] if the allocator refuses the matrix.
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight, or the negation of one, overflows `i64`.
    /// - [`Error::Internal`] on disagreement between the two algorithms (a bug).
    pub fn minimal_network(&self, budget: &Budget) -> Result<Verdict<MinimalNetwork>, Error> {
        let mut meter = budget.meter();

        if let Some(dist) = floyd_warshall::run(self, budget, &mut meter)? {
            return MinimalNetwork::new(self.time_points, dist).map(Verdict::Consistent);
        }

        match bellman_ford::run(self, &mut meter)? {
            Verdict::Inconsistent(cycle) => Ok(Verdict::Inconsistent(cycle)),
            Verdict::Consistent(_) => Err(Error::Internal),
        }
    }

    /// Appends one distance-graph edge.
    fn push_edge(&mut self, from: TimePoint, to: TimePoint, weight: Ticks, source: CycleEdge) {
        self.edges.push(Edge {
            from: from.0,
            to: to.0,
            weight,
            source,
        });
    }
}

/// Converts a collection length into a step count.
///
/// # Errors
///
/// [`Error::Overflow`] if the length does not fit `u64`.
fn steps(len: usize) -> Result<u64, Error> {
    u64::try_from(len).map_err(|_| Error::Overflow)
}
