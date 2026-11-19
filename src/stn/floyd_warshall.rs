//! All-pairs shortest paths with Floyd-Warshall.
//!
//! Inconsistency is detected without letting distances diverge: a negative cycle whose
//! highest-numbered vertex is `k` already shows as a negative `d[k][k]` before round `k`, and
//! before that point every entry is an exact restricted shortest-path length. The algorithm
//! stops at the first negative diagonal entry, so checked arithmetic only fails when a genuine
//! shortest path overflows `i64`.

use alloc::vec::Vec;

use super::{Stn, steps};
use crate::budget::Meter;
use crate::{Budget, Error, Ticks};

/// Row-major `n * n` distance matrix; `None` is `+infinity`.
type Matrix = Vec<Option<Ticks>>;

/// Runs Floyd-Warshall.
///
/// Returns the distance matrix, or `None` if the network is inconsistent.
///
/// # Errors
///
/// [`Error::MemoryBudgetExceeded`], [`Error::OutOfMemory`], [`Error::BudgetExhausted`] or
/// [`Error::Overflow`].
pub(super) fn run(stn: &Stn, budget: &Budget, meter: &mut Meter) -> Result<Option<Matrix>, Error> {
    let n = stn.time_points;
    let cells = n.checked_mul(n).ok_or(Error::MemoryBudgetExceeded)?;
    budget.check_cells(cells)?;

    meter.spend(steps(stn.edges.len())?)?;
    let mut dist = initial_matrix(stn, cells)?;

    let round_steps = steps(cells)?;

    for k in 0..n {
        if is_negative(dist[k * n + k]) {
            return Ok(None);
        }

        meter.spend(round_steps)?;
        relax_through(&mut dist, n, k)?;
    }

    if (0..n).any(|i| is_negative(dist[i * n + i])) {
        return Ok(None);
    }

    Ok(Some(dist))
}

/// Builds the matrix of direct edges: zero diagonal, the tightest parallel edge per pair.
///
/// # Errors
///
/// [`Error::OutOfMemory`] if the allocation fails.
fn initial_matrix(stn: &Stn, cells: usize) -> Result<Matrix, Error> {
    let n = stn.time_points;
    let mut dist = Vec::new();
    dist.try_reserve_exact(cells)
        .map_err(|_| Error::OutOfMemory)?;
    dist.resize(cells, None);

    for i in 0..n {
        dist[i * n + i] = Some(Ticks::ZERO);
    }

    for edge in &stn.edges {
        let cell = &mut dist[edge.from * n + edge.to];

        if cell.is_none_or(|d| edge.weight < d) {
            *cell = Some(edge.weight);
        }
    }

    Ok(dist)
}

/// Performs round `k`: shortens every `i -> j` path through `k`.
///
/// # Errors
///
/// [`Error::Overflow`] if a path length overflows.
fn relax_through(dist: &mut Matrix, n: usize, k: usize) -> Result<(), Error> {
    for i in 0..n {
        let Some(d_ik) = dist[i * n + k] else {
            continue;
        };

        for j in 0..n {
            let Some(d_kj) = dist[k * n + j] else {
                continue;
            };

            let candidate = d_ik.checked_add(d_kj).ok_or(Error::Overflow)?;
            let cell = &mut dist[i * n + j];

            if cell.is_none_or(|d| candidate < d) {
                *cell = Some(candidate);
            }
        }
    }

    Ok(())
}

/// Whether a matrix entry is a finite negative distance.
fn is_negative(d: Option<Ticks>) -> bool {
    d.is_some_and(|d| d < Ticks::ZERO)
}
