//! Explanations of non-controllability: semi-reducible negative cycles.

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

/// A semi-reducible negative cycle: requirement bounds and contingent extremes that no
/// dynamic strategy can satisfy together.
///
/// Edges are in traversal order: the head of each edge is the tail of the next, and the head
/// of the last is the tail of the first. Summing the edge weights (lower-case edges at the
/// minimum duration, upper-case edges at minus the maximum) gives [`Self::weight`], which is
/// strictly negative. Morris (2006) showed that an STNU is dynamically controllable iff its
/// labelled distance graph has no such cycle: the lower-case edges on it can be reduced away
/// (the agent cannot exploit an early end it has not seen yet), which leaves a negative cycle
/// of requirements and waits.
///
/// The cycle is a certificate in the sense of [`NegativeCycle`](crate::NegativeCycle): keeping
/// only the requirement bounds it names (and all contingent links) still gives a network that
/// is not dynamically controllable, which the tests check against independent checkers. It is
/// not necessarily simple or minimal.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct SemiReducibleCycle {
    /// Edges in traversal order. Never empty.
    edges: Vec<LabelledEdge>,
    /// Total weight of the cycle. Always negative.
    weight: Ticks,
}

impl SemiReducibleCycle {
    /// Creates a cycle from its edges and total weight.
    pub(crate) fn new(edges: Vec<LabelledEdge>, weight: Ticks) -> Self {
        Self { edges, weight }
    }

    /// Edges in traversal order.
    #[must_use]
    pub fn edges(&self) -> &[LabelledEdge] {
        &self.edges
    }

    /// Total cycle weight; negative.
    #[must_use]
    pub fn weight(&self) -> Ticks {
        self.weight
    }

    /// The contingent links whose extreme durations the cycle relies on, in cycle order,
    /// possibly repeated.
    pub fn contingent_links(&self) -> impl Iterator<Item = ContingentId> + '_ {
        self.edges.iter().filter_map(|e| match *e {
            LabelledEdge::LowerCase(id) | LabelledEdge::UpperCase(id) => Some(id),
            LabelledEdge::Requirement(_) => None,
        })
    }
}
