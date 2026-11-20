//! Explanations of non-controllability: semi-reducible negative cycles.

use alloc::vec;
use alloc::vec::Vec;

use super::{ContingentId, Stnu};
use crate::stn::{Bound, CycleEdge};
use crate::{Error, Ticks, TimePoint};

/// One edge of the labelled distance graph of an [`Stnu`], named by what supplied it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LabelledEdge {
    /// A requirement bound, as in an STN [`NegativeCycle`](crate::NegativeCycle).
    Requirement(CycleEdge),
    /// The lower-case edge `A -> C` of weight `lo` of a contingent link: the possibility that
    /// the duration takes its minimum.
    LowerCase(ContingentId),
    /// The upper-case edge `C -> A` of weight `-hi` of a contingent link: the possibility that
    /// the duration takes its maximum.
    UpperCase(ContingentId),
}

impl LabelledEdge {
    /// The edge as `(tail, head, weight)` in `stnu`, meaning `t_head - t_tail <= weight` for
    /// requirements and the extreme durations for the labelled edges.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if the constraint or link does not belong to `stnu`.
    /// - [`Error::Overflow`] if a bound cannot be negated.
    pub fn resolve(self, stnu: &Stnu) -> Result<(TimePoint, TimePoint, Ticks), Error> {
        match self {
            Self::Requirement(edge) => {
                let c = stnu
                    .requirements()
                    .constraint(edge.constraint)
                    .ok_or(Error::UnknownTimePoint)?;
                let missing = Error::UnknownTimePoint;

                match edge.bound {
                    Bound::Upper => Ok((c.from, c.to, c.hi.ok_or(missing)?)),
                    Bound::Lower => {
                        let lo = c.lo.ok_or(missing)?;
                        Ok((c.to, c.from, lo.checked_neg().ok_or(Error::Overflow)?))
                    }
                }
            }
            Self::LowerCase(id) => {
                let link = stnu.links().get(id.0).ok_or(Error::UnknownTimePoint)?;
                Ok((link.activation, link.contingent, link.lo))
            }
            Self::UpperCase(id) => {
                let link = stnu.links().get(id.0).ok_or(Error::UnknownTimePoint)?;
                let weight = link.hi.checked_neg().ok_or(Error::Overflow)?;
                Ok((link.contingent, link.activation, weight))
            }
        }
    }
}

/// One step of a [`SemiReducibleCycle`] or of a [`DerivedEdge`] in compact form.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CycleStep {
    /// An original edge of the labelled distance graph.
    Edge(LabelledEdge),
    /// A derived ordinary edge, by index into [`SemiReducibleCycle::derived`].
    Derived(usize),
}

/// An ordinary edge the checker derived (`t_to - t_from <= weight` in every dynamic execution),
/// with the path of steps it summarises.
///
/// Derived edges only refer to derived edges with smaller indices, so expanding one always
/// terminates.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct DerivedEdge {
    /// Tail.
    pub from: TimePoint,
    /// Head.
    pub to: TimePoint,
    /// Sum of the expanded path's weights.
    pub weight: Ticks,
    /// The summarised path from `from` to `to`, in traversal order. Never empty.
    pub steps: Vec<CycleStep>,
}

/// A semi-reducible negative cycle: requirement bounds and contingent extremes that no
/// dynamic strategy can satisfy together.
///
/// Stored in compact form: the top-level [`Self::steps`] chain like an STN cycle, and each
/// [`CycleStep::Derived`] stands for a [`DerivedEdge`] listed once in [`Self::derived`], whose
/// own steps may again be derived. The compact form has at most one entry per derived edge,
/// so it stays polynomial (`O(n^3)` steps for `n` points); [`Self::edges`] expands it lazily
/// into original edges, which can be exponentially many: an indivisible cycle can need
/// `3 * 2^k - 2` edges for `k` links (the *magic loops* of Hunsberger, ICAART 2013), with
/// repetitions.
///
/// In the expansion the head of each edge is the tail of the next, and the head of the last is
/// the tail of the first. Summing the edge weights (lower-case edges at the minimum duration,
/// upper-case edges at minus the maximum) gives [`Self::weight`], which is strictly negative.
/// Morris (2006) showed that an STNU is dynamically controllable iff its labelled distance
/// graph has no such cycle: the lower-case edges on it can be reduced away (the agent cannot
/// exploit an early end it has not seen yet), which leaves a negative cycle of requirements
/// and waits.
///
/// The cycle is a certificate in the sense of [`NegativeCycle`](crate::NegativeCycle): keeping
/// only the requirement bounds it names (and all contingent links) still gives a network that
/// is not dynamically controllable, which the tests check against independent checkers. It is
/// not necessarily simple or minimal.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct SemiReducibleCycle {
    /// Top-level steps in traversal order. Never empty.
    steps: Vec<CycleStep>,
    /// Derived edges referenced by the steps, each once, inner ones first.
    derived: Vec<DerivedEdge>,
    /// Total weight of the cycle. Always negative.
    weight: Ticks,
}

impl SemiReducibleCycle {
    /// Creates a cycle from its compact parts and total weight.
    pub(crate) fn new(steps: Vec<CycleStep>, derived: Vec<DerivedEdge>, weight: Ticks) -> Self {
        Self {
            steps,
            derived,
            weight,
        }
    }

    /// Top-level steps in traversal order.
    #[must_use]
    pub fn steps(&self) -> &[CycleStep] {
        &self.steps
    }

    /// The derived edges [`CycleStep::Derived`] refers to.
    #[must_use]
    pub fn derived(&self) -> &[DerivedEdge] {
        &self.derived
    }

    /// Total cycle weight; negative.
    #[must_use]
    pub fn weight(&self) -> Ticks {
        self.weight
    }

    /// The original edges in traversal order, expanding derived edges lazily (memory
    /// proportional to the nesting depth).
    ///
    /// The count can be exponential in the number of links; see [`Self::expanded_len`].
    pub fn edges(&self) -> impl Iterator<Item = LabelledEdge> + '_ {
        let mut stack = vec![self.steps.iter()];

        core::iter::from_fn(move || {
            loop {
                let top = stack.last_mut()?;

                match top.next() {
                    None => {
                        stack.pop();
                    }
                    Some(CycleStep::Edge(e)) => return Some(*e),
                    Some(CycleStep::Derived(i)) => stack.push(self.derived[*i].steps.iter()),
                }
            }
        })
    }

    /// Number of original edges [`Self::edges`] yields, or `None` if it exceeds `usize`.
    #[must_use]
    pub fn expanded_len(&self) -> Option<usize> {
        let mut lens: Vec<usize> = Vec::with_capacity(self.derived.len());

        for d in &self.derived {
            let len = steps_len(&d.steps, &lens)?;
            lens.push(len);
        }

        steps_len(&self.steps, &lens)
    }

    /// The distinct contingent links whose extreme durations the cycle relies on, ascending.
    #[must_use]
    pub fn contingent_links(&self) -> Vec<ContingentId> {
        let all = self.derived.iter().flat_map(|d| d.steps.iter());
        let mut links: Vec<ContingentId> = self
            .steps
            .iter()
            .chain(all)
            .filter_map(|s| match *s {
                CycleStep::Edge(LabelledEdge::LowerCase(id) | LabelledEdge::UpperCase(id)) => {
                    Some(id)
                }
                _ => None,
            })
            .collect();
        links.sort_unstable();
        links.dedup();
        links
    }
}

/// Expanded length of `steps`, given the expanded lengths of the derived edges before them.
fn steps_len(steps: &[CycleStep], lens: &[usize]) -> Option<usize> {
    steps.iter().try_fold(0usize, |acc, s| match *s {
        CycleStep::Edge(_) => acc.checked_add(1),
        CycleStep::Derived(i) => acc.checked_add(*lens.get(i)?),
    })
}
