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
//!     stnu -->|"check_controllability (Morris 2014 backprop)"| c["Controllable or Uncontrollable"]
//!     v --> d["Dispatcher (onboard)"]
//! ```

mod backprop;
mod closure;

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
    /// No dynamic strategy satisfies every requirement in every situation.
    ///
    /// shortcut: no explanation (semi-reducible negative cycle) yet; add one when operators
    /// need to see which contingent links and requirements conflict.
    Uncontrollable,
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

    /// Checks dynamic controllability with Morris's backpropagation from negative nodes.
    ///
    /// Independent of [`Stnu::dispatchable`]; both must agree, which the tests check. Use this
    /// when only the verdict is needed: it keeps `O(n + m)` memory plus derived edges instead
    /// of dense matrices. See the `backprop` module docs for the cost.
    ///
    /// # Errors
    ///
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight overflows `i64`.
    pub fn check_controllability(&self, budget: &Budget) -> Result<Controllability<()>, Error> {
        backprop::run(self, budget)
    }

    /// Checks dynamic controllability and derives a dispatchable extended STNU.
    ///
    /// Computes the closure of the labelled distance graph under the edge-generation rules of
    /// Morris and Muscettola (no-case, upper-case, lower-case, cross-case, label removal) in
    /// matrix form, checking the all-max projection for negative cycles after each round. The
    /// result keeps every ordinary edge and every *wait* (upper-case edge) that is tighter than
    /// the ordinary edge in parallel, which a [`Dispatcher`](crate::Dispatcher) executes.
    ///
    /// Each round costs `O(n^3 + n^2 k)` steps and `2 n^2 + n k` cells for `n` points and `k`
    /// links. shortcut: no polynomial bound on the number of rounds is proven here; values only
    /// decrease, so rounds are finite, and the [`Budget`] bounds them. Upgrade to the
    /// Hunsberger-Posenato `FD_STNU` algorithm if a proven bound matters.
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if the matrices exceed [`Budget::max_cells`].
    /// - [`Error::OutOfMemory`] if the allocator refuses them.
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight overflows `i64`.
    pub fn dispatchable(
        &self,
        budget: &Budget,
    ) -> Result<Controllability<DispatchableNetwork>, Error> {
        closure::run(self, budget)
    }
}
