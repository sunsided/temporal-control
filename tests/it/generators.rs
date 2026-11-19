//! Proptest strategies that produce STN specifications.

use proptest::prelude::*;
use temporal_control::{Error, Stn, Ticks, TimePoint};

/// A constraint as plain data: `(from, to, lo, hi)` meaning `lo <= t_to - t_from <= hi`.
pub type ConstraintSpec = (usize, usize, Option<i64>, Option<i64>);

/// An STN as plain data, so failing cases shrink and print readably.
#[derive(Debug, Clone)]
pub struct StnSpec {
    /// Number of time points. At least one.
    pub n: usize,
    /// Constraints; endpoints are below `n`.
    pub constraints: Vec<ConstraintSpec>,
}

impl StnSpec {
    /// Builds the [`Stn`], or returns the first construction error.
    pub fn build(&self) -> Result<Stn, Error> {
        let mut stn = Stn::new();
        for _ in 0..self.n {
            stn.add_time_point();
        }

        for &(from, to, lo, hi) in &self.constraints {
            stn.add_constraint(TimePoint(from), TimePoint(to), lo.map(Ticks), hi.map(Ticks))?;
        }

        Ok(stn)
    }
}

/// Random networks with small weights: self-loops, parallel edges, `lo > hi`, disconnected parts
/// and zero-weight cycles all occur. Skews towards inconsistent for dense inputs.
pub fn random_stn() -> impl Strategy<Value = StnSpec> {
    (1usize..=7).prop_flat_map(|n| {
        let bound = proptest::option::weighted(0.8, -20i64..=20);
        let constraint = (0..n, 0..n, bound.clone(), bound);
        proptest::collection::vec(constraint, 0..14)
            .prop_map(move |constraints| StnSpec { n, constraints })
    })
}

/// Networks consistent by construction: every interval contains the difference of a hidden
/// random schedule.
pub fn consistent_stn() -> impl Strategy<Value = StnSpec> {
    (1usize..=7).prop_flat_map(|n| {
        let schedule = proptest::collection::vec(-50i64..=50, n);
        let slack = proptest::option::weighted(0.8, 0i64..=10);
        let constraint = (0..n, 0..n, slack.clone(), slack);
        let constraints = proptest::collection::vec(constraint, 0..14);
        (schedule, constraints).prop_map(move |(times, raw)| StnSpec {
            n,
            constraints: raw
                .into_iter()
                .map(|(i, j, lo_slack, hi_slack)| {
                    let d = times[j] - times[i];
                    (i, j, lo_slack.map(|s| d - s), hi_slack.map(|s| d + s))
                })
                .collect(),
        })
    })
}

/// A mix of [`random_stn`] and [`consistent_stn`], so both verdicts are well represented.
pub fn any_stn() -> impl Strategy<Value = StnSpec> {
    prop_oneof![random_stn(), consistent_stn()]
}

/// Networks whose bounds are drawn from extreme values near `i64::MIN` and `i64::MAX`.
pub fn extreme_stn() -> impl Strategy<Value = StnSpec> {
    let extreme = prop_oneof![
        Just(i64::MIN),
        Just(i64::MIN + 1),
        Just(i64::MAX),
        Just(i64::MAX - 1),
        Just(i64::MAX / 2),
        Just(i64::MIN / 2),
        -2i64..=2,
    ];

    (1usize..=5).prop_flat_map(move |n| {
        let bound = proptest::option::of(extreme.clone());
        let constraint = (0..n, 0..n, bound.clone(), bound);
        proptest::collection::vec(constraint, 0..10)
            .prop_map(move |constraints| StnSpec { n, constraints })
    })
}
