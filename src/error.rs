//! Error model.

use core::fmt;

/// Reasons an algorithm could not produce a verdict.
///
/// An inconsistent network is not an error: it is a [`Verdict`](crate::Verdict) carrying an
/// explanation. Errors mean the question was not answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Error {
    /// A time point does not belong to this network.
    UnknownTimePoint,
    /// A contingent link needs `0 < lo <= hi`.
    InvalidContingentLink,
    /// A checked arithmetic operation on [`Ticks`](crate::Ticks) or a size overflowed.
    Overflow,
    /// The step budget ran out before the algorithm finished.
    BudgetExhausted,
    /// The algorithm needs more matrix cells than [`Budget::max_cells`](crate::Budget::max_cells).
    MemoryBudgetExceeded,
    /// The allocator could not provide the memory the budget allowed.
    OutOfMemory,
    /// Two independent checks disagreed, or an internal invariant broke. This is a bug.
    Internal,
    // New variants go last: reordering changes the discriminants (a semver break).
    /// An agent does not belong to this multi-agent network.
    UnknownAgent,
    /// The summaries handed to a group operation do not form one complete, current set: an
    /// agent is missing or duplicated, the groups or revisions disagree, or a shared
    /// constraint refers to a point its owner did not report.
    MismatchedSummaries,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            Self::UnknownTimePoint => "time point does not belong to this network",
            Self::UnknownAgent => "agent does not belong to this network",
            Self::MismatchedSummaries => "summaries do not form one complete, current set",
            Self::InvalidContingentLink => "contingent link bounds must satisfy 0 < lo <= hi",
            Self::Overflow => "tick arithmetic overflowed",
            Self::BudgetExhausted => "step budget exhausted",
            Self::MemoryBudgetExceeded => "memory budget exceeded",
            Self::OutOfMemory => "allocation failed",
            Self::Internal => "internal invariant violated",
        };

        f.write_str(msg)
    }
}

impl core::error::Error for Error {}
