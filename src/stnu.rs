//! Simple Temporal Networks with Uncertainty (STNU) and dynamic controllability.
//!
//! An STNU is an [`Stn`] of *requirement* constraints plus *contingent links* `(A, lo, hi, C)`:
//! once the activation point `A` happens at `a`, nature makes the contingent point `C` happen at
//! some time in `[a + lo, a + hi]`, and the agent learns it only then. An STNU is *dynamically
//! controllable* (DC) if the agent can execute its points, reacting to observations as they
//! arrive, so that every requirement holds for every outcome of the contingent links.
//!
//! # Reaction-time semantics
//!
//! Time is integer [`Ticks`]. An observation at tick `t` may influence decisions at the same
//! tick `t` (*instantaneous reaction at tick granularity*): within one tick, nature's contingent
//! events come first, then the agent acts. This is the semantics under which the edge-generation
//! rules used here are exact; the integration tests check it against a brute-force game solver.
//! It requires the executive to deliver observations before it asks for executions in a tick,
//! as [`Dispatcher::step`](crate::Dispatcher::step) does. A system that can only react one tick
//! later needs the stricter variant, which is not implemented (see `docs/design.md`).
//!
//! ```mermaid
//! graph LR
//!     stnu["Stnu"] -->|"dispatchable (labelled closure)"| v["Controllable(DispatchableNetwork) or Uncontrollable"]
//!     stnu -->|"check_controllability (Morris 2014, O(n^3))"| c["Controllable or Uncontrollable"]
//!     stnu -->|"check_controllability_with (per-label backprop)"| c
//!     v --> d["Dispatcher (onboard)"]
//!     c -->|"Uncontrollable"| e["SemiReducibleCycle"]
//! ```
//!
//! Every checker explains a negative verdict with a [`SemiReducibleCycle`]. The checkers other
//! than [`DcAlgorithm::Morris2014`] take it from that checker, and report [`Error::Internal`]
//! if it disagrees, so a negative verdict is always confirmed by two algorithms.

mod backprop;
mod closure;
mod explanation;
mod morris;

pub use explanation::{LabelledEdge, SemiReducibleCycle};

use alloc::vec::Vec;

use crate::dispatch::DispatchableNetwork;
use crate::stn::{ConstraintId, Stn};
use crate::{Budget, Error, Ticks, TimePoint};

/// Handle of a contingent link inside one [`Stnu`], in insertion order starting at zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ContingentId(pub usize);

/// A contingent link: `contingent` happens between `lo` and `hi` ticks after `activation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ContingentLink {
    /// The point that starts the uncontrollable duration. Any time point, contingent ones
    /// included.
    pub activation: TimePoint,
    /// The point nature executes. Created together with the link; no other link targets it.
    pub contingent: TimePoint,
    /// Shortest duration. Positive.
    pub lo: Ticks,
    /// Longest duration. At least `lo`.
    pub hi: Ticks,
}

/// Outcome of a dynamic-controllability check that answered the question.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Controllability<T> {
    /// The network is dynamically controllable; the payload is a derived structure.
    Controllable(T),
    /// No dynamic strategy satisfies every requirement in every situation; the payload names
    /// the requirements and contingent extremes that conflict.
    Uncontrollable(SemiReducibleCycle),
}

/// Which dynamic-controllability checker [`Stnu::check_controllability_with`] runs.
///
/// All of them decide the same question under the same (instantaneous-reaction) semantics;
/// they differ in cost and serve as cross-checks for each other.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum DcAlgorithm {
    /// Morris (2014): backpropagation from negative nodes, one propagation per node,
    /// `O(n m + n^3)` time, explains negative verdicts itself.
    #[default]
    Morris2014,
    /// Morris-style backpropagation without normalisation: one propagation per kind of
    /// negative incoming edge, `O(n (k + 1) (m + n^2) log n)`. Kept as an independent
    /// cross-check.
    PerLabelBackprop,
}

/// A Simple Temporal Network with Uncertainty.
///
/// # Examples
///
/// ```
/// use temporal_control::{Budget, Controllability, Stnu, Ticks};
///
/// // Warm-up takes 5..15 ticks; analysis must start within 2 ticks of its end.
/// let mut stnu = Stnu::new();
/// let start = stnu.add_time_point().expect("fits");
/// let warm = stnu.add_contingent_link(start, Ticks(5), Ticks(15)).expect("valid");
/// let analysis = stnu.add_time_point().expect("fits");
/// stnu.add_constraint(warm, analysis, Some(Ticks(0)), Some(Ticks(2))).expect("exists");
///
/// let verdict = stnu.dispatchable(&Budget::UNLIMITED).expect("within budget");
/// assert!(matches!(verdict, Controllability::Controllable(_)));
/// ```
#[derive(Debug, Clone, Default)]
pub struct Stnu {
    /// Time points and requirement constraints.
    requirements: Stn,
    /// Contingent links in insertion order; index is the [`ContingentId`].
    links: Vec<ContingentLink>,
}

impl Stnu {
    /// Creates an empty network.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds an executable time point.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if the number of time points would exceed `usize::MAX`.
    pub fn add_time_point(&mut self) -> Result<TimePoint, Error> {
        self.requirements.add_time_point()
    }

    /// Adds a contingent link from `activation` and returns its new contingent time point.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if `activation` is not in this network.
    /// - [`Error::InvalidContingentLink`] unless `0 < lo <= hi`.
    /// - [`Error::Overflow`] if the number of time points would exceed `usize::MAX`.
    pub fn add_contingent_link(
        &mut self,
        activation: TimePoint,
        lo: Ticks,
        hi: Ticks,
    ) -> Result<TimePoint, Error> {
        if activation.0 >= self.requirements.time_point_count() {
            return Err(Error::UnknownTimePoint);
        }

        if lo <= Ticks::ZERO || hi < lo {
            return Err(Error::InvalidContingentLink);
        }

        let contingent = self.requirements.add_time_point()?;
        self.links.push(ContingentLink {
            activation,
            contingent,
            lo,
            hi,
        });
        Ok(contingent)
    }

    /// Adds the requirement `lo <= t_to - t_from <= hi`. See [`Stn::add_constraint`].
    ///
    /// # Errors
    ///
    /// As for [`Stn::add_constraint`].
    pub fn add_constraint(
        &mut self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
    ) -> Result<ConstraintId, Error> {
        self.requirements.add_constraint(from, to, lo, hi)
    }

    /// Time points and requirement constraints, without the contingent links.
    #[must_use]
    pub fn requirements(&self) -> &Stn {
        &self.requirements
    }

    /// All contingent links, indexed by [`ContingentId`].
    #[must_use]
    pub fn links(&self) -> &[ContingentLink] {
        &self.links
    }

    /// Number of time points, executable and contingent.
    #[must_use]
    pub fn time_point_count(&self) -> usize {
        self.requirements.time_point_count()
    }

    /// Checks dynamic controllability with Morris's `O(n^3)` backpropagation from negative
    /// nodes ([`DcAlgorithm::Morris2014`]).
    ///
    /// Independent of [`Stnu::dispatchable`]; both must agree, which the tests check. Use this
    /// when only the verdict (and an explanation) is needed: it keeps `O(m + n^2)` memory in
    /// the worst case instead of dense matrices. See the `morris` module docs for the
    /// algorithm and the proof of its bound.
    ///
    /// [`Budget::max_cells`] must admit `(n + k)^2` cells, the worst case of derived edges.
    /// shortcut: the allocations themselves (adjacency lists, derived edges, search state)
    /// abort on allocator failure; make them fallible before running this onboard.
    ///
    /// # Examples
    ///
    /// ```
    /// use temporal_control::{Budget, Controllability, Stnu, Ticks};
    ///
    /// // X must precede the end of a [1, 10] activity by 1..2 ticks: impossible to schedule.
    /// let mut stnu = Stnu::new();
    /// let a = stnu.add_time_point().expect("fits");
    /// let c = stnu.add_contingent_link(a, Ticks(1), Ticks(10)).expect("valid");
    /// let x = stnu.add_time_point().expect("fits");
    /// stnu.add_constraint(x, c, Some(Ticks(1)), Some(Ticks(2))).expect("exists");
    ///
    /// let verdict = stnu.check_controllability(&Budget::UNLIMITED).expect("within budget");
    /// let Controllability::Uncontrollable(cycle) = verdict else {
    ///     panic!("not DC");
    /// };
    /// // X -> C: 2 (requirement), C -> A: -10 (C may take 10), A -> C: 1 (C may take 1),
    /// // C -> X: -1 (requirement): the agent would have to know C before it happens.
    /// assert!(cycle.weight() < Ticks::ZERO);
    /// assert!(cycle.contingent_links().count() >= 1);
    /// ```
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if `(n + k)^2` exceeds [`Budget::max_cells`].
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight overflows `i64`.
    pub fn check_controllability(&self, budget: &Budget) -> Result<Controllability<()>, Error> {
        morris::run(self, budget)
    }

    /// Checks dynamic controllability with the chosen `algorithm`.
    ///
    /// Every algorithm returns the same verdict. A negative verdict carries the explanation of
    /// [`DcAlgorithm::Morris2014`], which then also runs (against a fresh copy of `budget`).
    ///
    /// # Examples
    ///
    /// ```
    /// use temporal_control::{Budget, Controllability, DcAlgorithm, Stnu, Ticks};
    ///
    /// let mut stnu = Stnu::new();
    /// let a = stnu.add_time_point().expect("fits");
    /// let c = stnu.add_contingent_link(a, Ticks(1), Ticks(10)).expect("valid");
    /// let x = stnu.add_time_point().expect("fits");
    /// // X within 2 ticks after C: wait for C, then execute.
    /// stnu.add_constraint(c, x, Some(Ticks(0)), Some(Ticks(2))).expect("exists");
    ///
    /// for algorithm in [DcAlgorithm::Morris2014, DcAlgorithm::PerLabelBackprop] {
    ///     let verdict = stnu.check_controllability_with(algorithm, &Budget::UNLIMITED);
    ///     assert_eq!(verdict, Ok(Controllability::Controllable(())));
    /// }
    /// ```
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if an explanation is needed and `(n + k)^2` exceeds
    ///   [`Budget::max_cells`] (or, for [`DcAlgorithm::Morris2014`], always in that case).
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight overflows `i64`.
    /// - [`Error::Internal`] if the explaining checker disagrees with the chosen one.
    pub fn check_controllability_with(
        &self,
        algorithm: DcAlgorithm,
        budget: &Budget,
    ) -> Result<Controllability<()>, Error> {
        match algorithm {
            DcAlgorithm::Morris2014 => morris::run(self, budget),
            DcAlgorithm::PerLabelBackprop => backprop::run(self, budget),
        }
    }

    /// Checks dynamic controllability and derives a dispatchable extended STNU.
    ///
    /// Computes the closure of the labelled distance graph under the edge-generation rules of
    /// Morris and Muscettola (no-case, upper-case, lower-case, cross-case, label removal) in
    /// matrix form, checking the all-max projection for negative cycles after each round. The
    /// result keeps every ordinary edge and every *wait* (upper-case edge) that is tighter than
    /// the ordinary edge in parallel, which a [`Dispatcher`](crate::Dispatcher) executes.
    ///
    /// Each round costs `O(n^3 + n^2 k)` steps; the matrices take `3 n^2 + 2 n k` cells for `n`
    /// points and `k` links, all checked against [`Budget::max_cells`] up front. shortcut:
    /// termination and the number of rounds are not proven here (the tests only show agreement
    /// with a brute-force oracle on small networks); the [`Budget`] bounds the work. Upgrade to
    /// the Hunsberger-Posenato `FD_STNU` algorithm if a proven bound matters.
    ///
    /// A negative verdict carries the explanation of [`Stnu::check_controllability`], which
    /// then also runs (against a fresh copy of `budget`).
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if the matrices exceed [`Budget::max_cells`].
    /// - [`Error::OutOfMemory`] if the allocator refuses them.
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight overflows `i64`.
    /// - [`Error::Internal`] if the explaining checker finds the network DC.
    pub fn dispatchable(
        &self,
        budget: &Budget,
    ) -> Result<Controllability<DispatchableNetwork>, Error> {
        closure::run(self, budget)
    }
}
