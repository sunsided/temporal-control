//! Choosing the cuts of a temporal decoupling.
//!
//! The network is loaded into an [`IncrementalStn`], so the windows `[lower(x), upper(x)]` of
//! every point relative to the reference are always those of the current minimal network.
//! For each shared distance-graph edge `u -> v` with weight `w`, a value `c` is chosen and the
//! cuts `t_u - z >= c` and `t_v - z <= c + w` are added, in that order.
//!
//! Both additions keep the network consistent whenever `lower(v) - w <= c <= upper(u)`:
//!
//! - `t_u - z >= c` with `c <= upper(u)` leaves a schedule with `t_u >= c`.
//! - If `c >= lower(u)`, some schedule of the minimal network has `t_u = c`, and in it
//!   `t_v <= c + w`. Otherwise the first cut changed nothing, and `lower(v) <= c + w` holds by
//!   assumption. Either way the second cut leaves a schedule.
//!
//! `c` is the midpoint of `lo_c = max(lower(u), lower(v) - w)` and `hi_c = min(upper(u),
//! upper(v) - w)`, rounded down. Both lie in `[lower(v) - w, upper(u)]` (the minimal network
//! has `lower(v) - w <= upper(u)`), so the midpoint does too. When `lo_c <= hi_c`, the edge
//! still has slack and the midpoint splits it evenly. When `lo_c > hi_c`, the edge is already
//! implied by the windows, and the midpoint lies in `[upper(v) - w, lower(u)]`, so neither
//! cut tightens the global network. The cuts are still recorded: the windows may owe their
//! tightness to other agents' constraints, and each agent's local network must imply its side
//! on its own. An unbounded `lo_c` (or `hi_c`) is replaced by the other value, and `0` is used
//! when both are unbounded.

use alloc::vec::Vec;

use super::{Cut, REFERENCE, SharedConstraint};
use crate::budget::Meter;
use crate::stn::{Bound, IncrementalStn, Stn, Verdict, steps};
use crate::{Budget, Error, Ticks, TimePoint};

/// Chooses the cuts of every shared constraint in `shared`, in order, on `stn`.
///
/// `shared` refers to the points of `stn`, whose point `0` is the reference. Cuts on the
/// same point accumulate; none is dropped.
///
/// # Errors
///
/// - [`Error::MemoryBudgetExceeded`] if `n^2` exceeds [`Budget::max_cells`].
/// - [`Error::OutOfMemory`] if the allocator refuses the matrix.
/// - [`Error::BudgetExhausted`] if the steps of the whole computation exceed the budget.
/// - [`Error::Overflow`] if a path weight or a cut overflows `i64`.
/// - [`Error::Internal`] if a cut is rejected, or the explanation search disagrees (a bug).
pub(super) fn choose(
    stn: &Stn,
    shared: &[SharedConstraint],
    budget: &Budget,
) -> Result<Verdict<Vec<Cut>>, Error> {
    let mut meter = budget.meter();
    let Some(mut inc) = load(stn, budget, &mut meter)? else {
        return explain(stn, &mut meter);
    };

    let edge_steps = steps(stn.time_point_count())?
        .saturating_pow(2)
        .saturating_mul(2);
    let mut cuts = Vec::new();

    for s in shared {
        for (u, v, w) in distance_edges(s) {
            meter.spend(edge_steps)?;
            cut_edge(&mut inc, s, u, v, w, budget, &mut cuts)?;
        }
    }

    Ok(Verdict::Consistent(cuts))
}

/// Loads `stn` into an incremental network; `None` if a constraint is rejected.
///
/// Charges `meter` with an upper bound of the steps each insertion costs.
///
/// # Errors
///
/// As for [`IncrementalStn::add_time_point`] and [`IncrementalStn::add_constraint`], with the
/// steps of all insertions metered together.
fn load(stn: &Stn, budget: &Budget, meter: &mut Meter) -> Result<Option<IncrementalStn>, Error> {
    let n = stn.time_point_count();
    let mut inc = IncrementalStn::new();

    for k in 1..=n {
        meter.spend(steps(k)?.saturating_pow(2))?;
        inc.add_time_point(budget)?;
    }

    let edge_steps = steps(n)?.saturating_pow(2).saturating_mul(2);

    for c in stn.constraints() {
        meter.spend(edge_steps)?;

        if let Verdict::Inconsistent(_) = inc.add_constraint(c.from, c.to, c.lo, c.hi, budget)? {
            return Ok(None);
        }
    }

    Ok(Some(inc))
}

/// Explains why `stn` is inconsistent, with the steps left in `meter`.
///
/// # Errors
///
/// [`Error::BudgetExhausted`] or [`Error::Overflow`] from Bellman-Ford, or [`Error::Internal`]
/// if it finds `stn` consistent.
fn explain(stn: &Stn, meter: &mut Meter) -> Result<Verdict<Vec<Cut>>, Error> {
    let remaining = Budget {
        max_steps: meter.remaining(),
        max_cells: usize::MAX,
    };

    match stn.check_consistency(&remaining)? {
        Verdict::Inconsistent(cycle) => Ok(Verdict::Inconsistent(cycle)),
        Verdict::Consistent(_) => Err(Error::Internal),
    }
}

/// The distance-graph edges `(u, v, w)`, meaning `t_v - t_u <= w`, of a shared constraint.
fn distance_edges(s: &SharedConstraint) -> impl Iterator<Item = (TimePoint, TimePoint, Ticks)> {
    let c = s.constraint;
    let upper = c.hi.map(|hi| (c.from, c.to, hi));
    let lower = c.lo.and_then(Ticks::checked_neg).map(|w| (c.to, c.from, w));

    upper.into_iter().chain(lower)
}

/// Adds the two cuts of the shared edge `u -> v` with weight `w` and records them in `cuts`.
///
/// `u`, `v` and `w` keep the names of the module docs.
///
/// # Errors
///
/// - [`Error::Overflow`] if `c` or `c + w` does not fit `i64`, or an update overflows.
/// - [`Error::BudgetExhausted`] if an update does not fit the step budget.
/// - [`Error::Internal`] if a cut is rejected (a bug: see the module docs).
#[expect(
    clippy::many_single_char_names,
    reason = "u, v, w and c are the names of the module docs"
)]
fn cut_edge(
    inc: &mut IncrementalStn,
    s: &SharedConstraint,
    u: TimePoint,
    v: TimePoint,
    w: Ticks,
    budget: &Budget,
    cuts: &mut Vec<Cut>,
) -> Result<(), Error> {
    let window = |x| (inc.lower(REFERENCE, x), inc.upper(REFERENCE, x));
    let c = split(window(u), window(v), w)?;
    let lower = Cut {
        shared: s.id,
        point: u,
        bound: Bound::Lower,
        value: c,
    };
    let upper = Cut {
        shared: s.id,
        point: v,
        bound: Bound::Upper,
        value: c.checked_add(w).ok_or(Error::Overflow)?,
    };

    for cut in [lower, upper] {
        let constraint = cut.constraint();

        if let Verdict::Inconsistent(_) = inc.add_constraint(
            constraint.from,
            constraint.to,
            constraint.lo,
            constraint.hi,
            budget,
        )? {
            return Err(Error::Internal);
        }

        cuts.push(cut);
    }

    Ok(())
}

/// The cut value `c` of a shared edge `u -> v` with weight `w`, given both windows.
///
/// # Errors
///
/// [`Error::Overflow`] if the midpoint does not fit `i64`.
fn split(
    (lower_u, upper_u): (Option<Ticks>, Option<Ticks>),
    (lower_v, upper_v): (Option<Ticks>, Option<Ticks>),
    w: Ticks,
) -> Result<Ticks, Error> {
    let shift = |t: Option<Ticks>| t.map(|t| i128::from(t.0) - i128::from(w.0));
    let lo_c = max_present(lower_u.map(|t| i128::from(t.0)), shift(lower_v));
    let hi_c = min_present(upper_u.map(|t| i128::from(t.0)), shift(upper_v));

    let c = match (lo_c, hi_c) {
        (Some(lo), Some(hi)) => (lo + hi).div_euclid(2),
        (Some(bound), None) | (None, Some(bound)) => bound,
        (None, None) => 0,
    };

    i64::try_from(c).map(Ticks).map_err(|_| Error::Overflow)
}

/// The larger of the present values; `None` if neither is (both bounds at `-infinity`).
fn max_present(a: Option<i128>, b: Option<i128>) -> Option<i128> {
    a.into_iter().chain(b).max()
}

/// The smaller of the present values; `None` if neither is (both bounds at `+infinity`).
fn min_present(a: Option<i128>, b: Option<i128>) -> Option<i128> {
    a.into_iter().chain(b).min()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Slack on both sides is split at the midpoint.
    #[test]
    fn split_halves_slack() {
        // u in [0, 10], v in [0, 30], v - u <= 10: c in [0, 10] -> 5.
        let c = split(
            (Some(Ticks(0)), Some(Ticks(10))),
            (Some(Ticks(0)), Some(Ticks(30))),
            Ticks(10),
        );
        assert_eq!(c, Ok(Ticks(5)));
    }

    /// An already implied edge gets a cut that tightens neither window.
    #[test]
    fn split_implied_edge_keeps_windows() {
        // u in [20, 30], v in [0, 25], v - u <= 10: implied; c in [15, 20].
        let c = split(
            (Some(Ticks(20)), Some(Ticks(30))),
            (Some(Ticks(0)), Some(Ticks(25))),
            Ticks(10),
        )
        .expect("fits");
        assert!(Ticks(15) <= c && c <= Ticks(20));
    }

    /// Unbounded windows fall back to the finite side, or to zero.
    #[test]
    fn split_unbounded() {
        assert_eq!(split((None, None), (None, None), Ticks(3)), Ok(Ticks(0)));
        assert_eq!(
            split((None, None), (None, Some(Ticks(9))), Ticks(3)),
            Ok(Ticks(6))
        );
        assert_eq!(
            split((Some(Ticks(4)), None), (None, None), Ticks(3)),
            Ok(Ticks(4))
        );
    }

    /// A midpoint beyond `i64` is an overflow, not a wrap.
    #[test]
    fn split_overflow() {
        let c = split((None, None), (Some(Ticks(i64::MIN)), None), Ticks(1));
        assert_eq!(c, Err(Error::Overflow));
    }
}
