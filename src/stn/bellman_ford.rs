//! Consistency checking with Bellman-Ford and negative-cycle extraction.
//!
//! A virtual source with a zero-weight edge to every time point is simulated by starting all
//! distances at zero. With `n` time points, every shortest path from the source uses at most
//! `n - 1` real edges, so a relaxation in pass `n` proves a negative cycle. Walking the
//! predecessor chain back `n` times from the last relaxed vertex lands on that cycle.

use alloc::vec;
use alloc::vec::Vec;

use super::{NegativeCycle, Schedule, Stn, Verdict, steps};
use crate::budget::Meter;
use crate::{Error, Ticks};

/// Per-vertex search state: tentative distances and the edge that last improved each one.
struct State {
    /// Tentative distance from the virtual source; also a candidate schedule.
    dist: Vec<Ticks>,
    /// Index into [`Stn::edges`] of the edge that set `dist`, or `None` if never improved.
    pred: Vec<Option<usize>>,
}

/// Runs Bellman-Ford over the whole distance graph.
///
/// # Errors
///
/// [`Error::BudgetExhausted`], [`Error::Overflow`], or [`Error::Internal`] on a broken invariant.
pub(super) fn run(stn: &Stn, meter: &mut Meter) -> Result<Verdict<Schedule>, Error> {
    let n = stn.time_points;
    let per_pass = steps(stn.edges.len())?;
    let mut state = State {
        dist: vec![Ticks::ZERO; n],
        pred: vec![None; n],
    };

    for pass in 0..n {
        meter.spend(per_pass)?;

        let Some(last) = relax_all(stn, &mut state)? else {
            break;
        };

        if pass + 1 == n {
            return extract_cycle(stn, &state.pred, last).map(Verdict::Inconsistent);
        }
    }

    Ok(Verdict::Consistent(Schedule { times: state.dist }))
}

/// Relaxes every edge once.
///
/// Returns the head of the last improved edge, or `None` if nothing changed.
///
/// # Errors
///
/// [`Error::Overflow`] if a tentative distance overflows.
fn relax_all(stn: &Stn, state: &mut State) -> Result<Option<usize>, Error> {
    let mut last = None;

    for (index, edge) in stn.edges.iter().enumerate() {
        let candidate = state.dist[edge.from]
            .checked_add(edge.weight)
            .ok_or(Error::Overflow)?;

        if candidate < state.dist[edge.to] {
            state.dist[edge.to] = candidate;
            state.pred[edge.to] = Some(index);
            last = Some(edge.to);
        }
    }

    Ok(last)
}

/// Extracts the negative cycle reachable backwards from `last` in the predecessor graph.
///
/// # Errors
///
/// [`Error::Internal`] if the predecessor chain is broken or the cycle is not negative;
/// [`Error::Overflow`] if the cycle weight overflows.
fn extract_cycle(stn: &Stn, pred: &[Option<usize>], last: usize) -> Result<NegativeCycle, Error> {
    let n = stn.time_points;
    let pred_edge = |v: usize| pred[v].map(|e| &stn.edges[e]).ok_or(Error::Internal);

    let mut start = last;

    for _ in 0..n {
        start = pred_edge(start)?.from;
    }

    let mut edges = Vec::new();
    let mut weight = Ticks::ZERO;
    let mut v = start;

    loop {
        let edge = pred_edge(v)?;
        edges.push(edge.source);
        weight = weight.checked_add(edge.weight).ok_or(Error::Overflow)?;
        v = edge.from;

        if v == start {
            break;
        }

        if edges.len() > n {
            return Err(Error::Internal);
        }
    }

    if weight >= Ticks::ZERO {
        return Err(Error::Internal);
    }

    edges.reverse();
    Ok(NegativeCycle::new(edges, weight))
}
