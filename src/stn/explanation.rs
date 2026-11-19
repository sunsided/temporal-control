//! Structured explanations of inconsistency.

use alloc::vec::Vec;

use super::ConstraintId;
use crate::Ticks;

/// Which bound of a constraint a distance-graph edge encodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bound {
    /// The upper bound `t_to - t_from <= hi`, edge `from -> to` with weight `hi`.
    Upper,
    /// The lower bound `t_to - t_from >= lo`, edge `to -> from` with weight `-lo`.
    Lower,
}

/// One edge of a negative cycle, named by the constraint bound that supplied it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CycleEdge {
    /// The constraint the edge comes from.
    pub constraint: ConstraintId,
    /// Which bound of that constraint.
    pub bound: Bound,
}

impl CycleEdge {
    /// Creates a cycle edge reference.
    pub(crate) const fn new(constraint: ConstraintId, bound: Bound) -> Self {
        Self { constraint, bound }
    }
}

/// A negative cycle in the distance graph: the constraints that jointly cannot hold.
///
/// Edges are in traversal order: the head of each edge is the tail of the next, and the head of
/// the last edge is the tail of the first. Summing the edge weights gives [`Self::weight`],
/// which is strictly negative.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct NegativeCycle {
    /// Edges in traversal order. Never empty.
    edges: Vec<CycleEdge>,
    /// Total weight of the cycle. Always negative.
    weight: Ticks,
}

impl NegativeCycle {
    /// Creates a cycle from its edges and total weight.
    pub(crate) fn new(edges: Vec<CycleEdge>, weight: Ticks) -> Self {
        Self { edges, weight }
    }

    /// Edges in traversal order.
    #[must_use]
    pub fn edges(&self) -> &[CycleEdge] {
        &self.edges
    }

    /// Total cycle weight; negative.
    #[must_use]
    pub fn weight(&self) -> Ticks {
        self.weight
    }
}
