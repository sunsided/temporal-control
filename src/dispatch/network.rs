//! Ground-side container for a dispatchable network.

use alloc::vec::Vec;

use super::{DispatchEdge, DispatchError, Dispatcher, PointKind, Wait};

/// A dispatchable network produced on the ground, ready to load into a [`Dispatcher`].
///
/// Produced by [`Stn::dispatchable`](crate::Stn::dispatchable) and
/// [`Stnu::dispatchable`](crate::Stnu::dispatchable). Point indices match the source network.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DispatchableNetwork {
    /// Kind of every time point, indexed by [`TimePoint`](crate::TimePoint).
    points: Vec<PointKind>,
    /// Edges the dispatcher propagates along.
    edges: Vec<DispatchEdge>,
    /// Waits of executable points on contingent points. Empty for STNs.
    waits: Vec<Wait>,
}

impl DispatchableNetwork {
    /// Bundles the parts of a dispatchable network.
    pub(crate) fn new(points: Vec<PointKind>, edges: Vec<DispatchEdge>, waits: Vec<Wait>) -> Self {
        Self {
            points,
            edges,
            waits,
        }
    }

    /// Kind of every time point.
    #[must_use]
    pub fn points(&self) -> &[PointKind] {
        &self.points
    }

    /// Dispatchable edges.
    #[must_use]
    pub fn edges(&self) -> &[DispatchEdge] {
        &self.edges
    }

    /// Waits on contingent points.
    #[must_use]
    pub fn waits(&self) -> &[Wait] {
        &self.waits
    }

    /// Adjacency entries a [`Dispatcher`] needs: two per edge plus one per wait.
    #[must_use]
    pub fn entry_count(&self) -> usize {
        2 * self.edges.len() + self.waits.len()
    }

    /// Loads this network into a fixed-capacity dispatcher.
    ///
    /// # Errors
    ///
    /// As for [`Dispatcher::new`].
    pub fn dispatcher<const N: usize, const E: usize>(
        &self,
    ) -> Result<Dispatcher<N, E>, DispatchError> {
        Dispatcher::new(&self.points, &self.edges, &self.waits)
    }
}
