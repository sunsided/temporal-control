//! Read access to a dispatchable network in any representation, and its validation.
//!
//! The dispatcher loads from slices ([`Slices`]) and from encoded bytes
//! ([`Encoded`](super::encoding::Encoded)). Both are read through [`Source`], so one validator
//! and one fill routine serve both, and neither needs intermediate arrays: the fixed-width
//! encoding gives random access to record `i` directly in the received bytes.

use super::{DispatchEdge, DispatchError, Index, PointKind, Wait, index};
use crate::Ticks;

/// Random access to the records of a dispatchable network.
///
/// Accessors return `None` for an index out of range or a record that cannot be read; the
/// validator treats that as [`DispatchError::InvalidNetwork`].
pub(super) trait Source {
    /// Number of time points.
    fn point_count(&self) -> usize;

    /// Number of edges.
    fn edge_count(&self) -> usize;

    /// Number of waits.
    fn wait_count(&self) -> usize;

    /// Kind of point `i`.
    fn point(&self, i: usize) -> Option<PointKind>;

    /// Edge `i`.
    fn edge(&self, i: usize) -> Option<DispatchEdge>;

    /// Wait `i`.
    fn wait(&self, i: usize) -> Option<Wait>;
}

/// A network given as three slices, as accepted by [`Dispatcher::load`](super::Dispatcher::load).
#[derive(Debug, Clone, Copy)]
pub(super) struct Slices<'a> {
    /// Kind of every point.
    pub(super) points: &'a [PointKind],
    /// Every edge.
    pub(super) edges: &'a [DispatchEdge],
    /// Every wait.
    pub(super) waits: &'a [Wait],
}

impl Source for Slices<'_> {
    fn point_count(&self) -> usize {
        self.points.len()
    }

    fn edge_count(&self) -> usize {
        self.edges.len()
    }

    fn wait_count(&self) -> usize {
        self.waits.len()
    }

    fn point(&self, i: usize) -> Option<PointKind> {
        self.points.get(i).copied()
    }

    fn edge(&self, i: usize) -> Option<DispatchEdge> {
        self.edges.get(i).copied()
    }

    fn wait(&self, i: usize) -> Option<Wait> {
        self.waits.get(i).copied()
    }
}

/// Number of adjacency entries `net` needs: two per edge plus one per wait.
///
/// # Errors
///
/// [`DispatchError::CapacityExceeded`] if the count overflows `usize`.
pub(super) fn entry_count(net: &impl Source) -> Result<usize, DispatchError> {
    net.edge_count()
        .checked_mul(2)
        .and_then(|e| e.checked_add(net.wait_count()))
        .ok_or(DispatchError::CapacityExceeded)
}

/// Validates `net` against capacities `n_cap` and `e_cap` and index type `I`.
///
/// # Errors
///
/// [`DispatchError::CapacityExceeded`], [`DispatchError::IndexTypeTooSmall`] or
/// [`DispatchError::InvalidNetwork`], checked in that order.
pub(super) fn validate<I: Index>(
    net: &impl Source,
    n_cap: usize,
    e_cap: usize,
) -> Result<(), DispatchError> {
    let n = net.point_count();
    let entries = entry_count(net)?;

    if n > n_cap || entries > e_cap {
        return Err(DispatchError::CapacityExceeded);
    }

    index::<I>(n)?;
    index::<I>(entries)?;

    let valid = (0..n).all(|p| valid_point(net, p))
        && (0..net.edge_count()).all(|i| net.edge(i).is_some_and(|e| valid_edge(&e, n)))
        && (0..net.wait_count()).all(|i| net.wait(i).is_some_and(|w| valid_wait(net, &w)));

    if !valid {
        return Err(DispatchError::InvalidNetwork);
    }

    Ok(())
}

/// Whether point `p` is well-formed, including an acyclic activation chain.
fn valid_point(net: &impl Source, p: usize) -> bool {
    let Some(kind) = net.point(p) else {
        return false;
    };

    let PointKind::Contingent { lo, hi, .. } = kind else {
        return true;
    };

    if lo <= Ticks::ZERO || hi < lo {
        return false;
    }

    let mut current = p;

    for _ in 0..net.point_count() {
        match net.point(current) {
            Some(PointKind::Contingent { activation, .. }) => current = activation.0,
            Some(PointKind::Executable) => return true,
            None => return false,
        }
    }

    false
}

/// Whether an edge has valid endpoints and is not a negative self-loop.
fn valid_edge(e: &DispatchEdge, n: usize) -> bool {
    e.from.0 < n && e.to.0 < n && (e.from != e.to || e.weight >= Ticks::ZERO)
}

/// Whether a wait goes from an executable point to a contingent point.
fn valid_wait(net: &impl Source, w: &Wait) -> bool {
    matches!(net.point(w.waiter.0), Some(PointKind::Executable))
        && matches!(
            net.point(w.contingent.0),
            Some(PointKind::Contingent { .. })
        )
}
