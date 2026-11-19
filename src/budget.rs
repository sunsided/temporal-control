//! Compute and memory budgets.

use crate::Error;

/// Explicit compute and memory limits for one algorithm invocation.
///
/// Every algorithm in this crate takes a budget. When a limit would be exceeded the algorithm
/// stops and returns [`Error::BudgetExhausted`] or [`Error::MemoryBudgetExceeded`]; it never
/// returns a partial verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Budget {
    /// Maximum number of elementary steps (one edge relaxation or one matrix-cell update).
    pub max_steps: u64,
    /// Maximum number of distance-matrix cells an algorithm may allocate.
    ///
    /// Floyd-Warshall needs `n * n` cells for `n` time points.
    pub max_cells: usize,
}

impl Budget {
    /// A budget with no practical limit, for ground-side use and tests.
    pub const UNLIMITED: Self = Self {
        max_steps: u64::MAX,
        max_cells: usize::MAX,
    };

    /// Starts metering steps against this budget.
    #[cfg_attr(
        not(feature = "alloc"),
        expect(dead_code, reason = "used by alloc algorithms")
    )]
    pub(crate) const fn meter(&self) -> Meter {
        Meter {
            remaining: self.max_steps,
        }
    }

    /// Checks that `cells` matrix cells fit the memory budget.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`] when `cells` exceeds [`Budget::max_cells`].
    #[cfg_attr(
        not(feature = "alloc"),
        expect(dead_code, reason = "used by alloc algorithms")
    )]
    pub(crate) const fn check_cells(&self, cells: usize) -> Result<(), Error> {
        if cells > self.max_cells {
            return Err(Error::MemoryBudgetExceeded);
        }

        Ok(())
    }
}

/// Running step counter derived from a [`Budget`].
#[derive(Debug)]
#[cfg_attr(
    not(feature = "alloc"),
    expect(dead_code, reason = "used by alloc algorithms")
)]
pub(crate) struct Meter {
    /// Steps still available.
    remaining: u64,
}

impl Meter {
    /// Consumes `steps` steps.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] when fewer than `steps` steps remain.
    #[cfg_attr(
        not(feature = "alloc"),
        expect(dead_code, reason = "used by alloc algorithms")
    )]
    pub(crate) fn spend(&mut self, steps: u64) -> Result<(), Error> {
        self.remaining = self
            .remaining
            .checked_sub(steps)
            .ok_or(Error::BudgetExhausted)?;

        Ok(())
    }
}
