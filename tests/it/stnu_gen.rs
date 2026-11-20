//! Proptest strategies that produce small STNUs with a reference point and a horizon.

use proptest::prelude::*;
use temporal_control::{Stnu, Ticks, TimePoint};

/// A tiny STNU as plain data.
///
/// Points `0..n_exec` are executable, point 0 is the reference. Contingent link `i` creates
/// point `n_exec + i`. Every executable point `x > 0` is constrained to `0 <= x - t_0 <= horizon`
/// in [`StnuSpec::build`] and in [`StnuSpec::edges`], so both the checker and the oracle see it.
#[derive(Debug, Clone)]
pub struct StnuSpec {
    /// Number of executable points, the reference included. At least one.
    pub n_exec: usize,
    /// Contingent links `(activation, lo, hi)`; activation is a lower-numbered point.
    pub links: Vec<(usize, i64, i64)>,
    /// Requirements `(from, to, lo, hi)`.
    pub constraints: Vec<(usize, usize, Option<i64>, Option<i64>)>,
    /// Latest tick for executable points.
    pub horizon: i64,
}

impl StnuSpec {
    /// Total number of points.
    pub fn point_count(&self) -> usize {
        self.n_exec + self.links.len()
    }

    /// Index of the contingent point of link `i`.
    pub fn contingent(&self, i: usize) -> usize {
        self.n_exec + i
    }

    /// Requirement edges `(u, v, w)` meaning `t_v - t_u <= w`, horizon included.
    pub fn edges(&self) -> impl Iterator<Item = (usize, usize, i64)> + '_ {
        self.all_constraints().flat_map(|(u, v, lo, hi)| {
            hi.map(|h| (u, v, h))
                .into_iter()
                .chain(lo.map(|l| (v, u, -l)))
        })
    }

    /// Requirements plus horizon constraints.
    pub fn all_constraints(
        &self,
    ) -> impl Iterator<Item = (usize, usize, Option<i64>, Option<i64>)> + '_ {
        let horizon = (1..self.n_exec).map(|x| (0, x, Some(0), Some(self.horizon)));
        self.constraints.iter().copied().chain(horizon)
    }

    /// Builds the [`Stnu`]: executables first, then one contingent point per link.
    pub fn build(&self) -> Stnu {
        let mut stnu = Stnu::new();

        for _ in 0..self.n_exec {
            stnu.add_time_point().unwrap();
        }

        for &(a, lo, hi) in &self.links {
            stnu.add_contingent_link(TimePoint(a), Ticks(lo), Ticks(hi))
                .unwrap();
        }

        for (u, v, lo, hi) in self.all_constraints() {
            stnu.add_constraint(TimePoint(u), TimePoint(v), lo.map(Ticks), hi.map(Ticks))
                .unwrap();
        }

        stnu
    }
}

/// Small random STNUs: up to 3 executable points and 2 contingent links, for the oracle.
pub fn small_stnu() -> impl Strategy<Value = StnuSpec> {
    stnu_up_to(3, 2, 5)
}

/// Larger random STNUs, too big for the oracle: up to 7 executable points and 4 links.
pub fn medium_stnu() -> impl Strategy<Value = StnuSpec> {
    stnu_up_to(7, 4, 6)
}

/// Random STNUs with at most `max_exec` executable points, `max_links` contingent links and
/// `max_constraints` requirements besides the horizon.
///
/// Half of the requirements bound a contingent point from above relative to an executable one,
/// because those create waits; uniform pairs rarely exercise the upper-case rule.
fn stnu_up_to(
    max_exec: usize,
    max_links: usize,
    max_constraints: usize,
) -> impl Strategy<Value = StnuSpec> {
    (1usize..=max_exec, 0usize..=max_links).prop_flat_map(move |(n_exec, k)| {
        let n = n_exec + k;
        let links = (0..k)
            .map(|i| (0..n_exec + i, 1i64..=2, 1i64..=6))
            .collect::<Vec<_>>()
            .prop_map(|raw| {
                raw.into_iter()
                    .map(|(a, lo, extra)| (a, lo, lo + extra))
                    .collect()
            });
        let bounds = |lo_range, hi_weight| {
            (
                proptest::option::weighted(0.6, lo_range),
                proptest::option::weighted(hi_weight, 0i64..=6),
            )
                .prop_map(|(lo, span): (Option<i64>, Option<i64>)| {
                    (lo, span.map(|s| lo.unwrap_or(0) + s))
                })
        };
        let any = ((0..n, 0..n), bounds(-4i64..=3, 0.6))
            .prop_map(|((u, v), (lo, hi))| (u, v, lo, hi))
            .boxed();
        // `t_c - t_x <= hi` with a small `hi` and a non-positive `lo` forces `x` to wait for `c`.
        // The reference point never waits, so waiters are drawn from `1..n_exec`.
        let waiting = if k == 0 || n_exec < 2 {
            any.clone()
        } else {
            (n_exec..n, 1..n_exec, bounds(-6i64..=0, 0.95))
                .prop_map(|(c, x, (lo, hi))| (x, c, lo, hi.map(|h| h.min(2))))
                .boxed()
        };
        let constraint = prop_oneof![any, waiting];
        let constraints = proptest::collection::vec(constraint, 1..=max_constraints);
        (links, constraints, 3i64..=7).prop_map(move |(links, constraints, horizon)| StnuSpec {
            n_exec,
            links,
            constraints,
            horizon,
        })
    })
}
