//! Time point handles.

/// Handle of a time point inside one network.
///
/// Handles are dense indices starting at zero, in creation order. A handle is only meaningful
/// for the network that created it, and for structures derived from that network (minimal and
/// dispatchable networks, dispatchers).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TimePoint(pub usize);
