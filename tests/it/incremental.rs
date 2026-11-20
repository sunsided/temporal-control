//! Incremental STN consistency, cross-checked against from-scratch recomputation.

use proptest::prelude::*;
use temporal_control::{Budget, Error, IncrementalStn, Stn, Ticks, TimePoint, Verdict};

use crate::generators::{StnSpec, any_stn, extreme_stn};
use crate::properties::{assert_valid_cycle, is_consistent};

/// Feeds every constraint of `spec` into an [`IncrementalStn`] and a reference [`Stn`] holding
/// only the accepted ones, and checks after each step that both agree.
fn replay(spec: &StnSpec) -> Result<(), TestCaseError> {
    let budget = Budget::UNLIMITED;
    let mut inc = IncrementalStn::new();
    let mut accepted = Stn::new();

    for _ in 0..spec.n {
        inc.add_time_point(&budget).unwrap();
        accepted.add_time_point().unwrap();
    }

    for &(from, to, lo, hi) in &spec.constraints {
        let (from, to, lo, hi) = (TimePoint(from), TimePoint(to), lo.map(Ticks), hi.map(Ticks));
        let mut candidate = accepted.clone();
        candidate.add_constraint(from, to, lo, hi).unwrap();
        let expected = is_consistent(&candidate.check_consistency(&budget).unwrap());

        match inc.add_constraint(from, to, lo, hi, &budget).unwrap() {
            Verdict::Consistent(_) => {
                prop_assert!(expected, "accepted an inconsistent constraint");
                accepted = candidate;
            }
            Verdict::Inconsistent(cycle) => {
                prop_assert!(!expected, "rejected a consistent constraint");
                assert_valid_cycle(&candidate, &cycle);
            }
        }

        let Verdict::Consistent(reference) = accepted.minimal_network(&budget).unwrap() else {
            panic!("accepted constraints must stay consistent");
        };
        prop_assert_eq!(&inc.minimal_network().unwrap(), &reference);
    }

    prop_assert_eq!(inc.stn().constraints(), accepted.constraints());
    Ok(())
}

/// Time points can be added between constraints; earlier distances are kept.
#[test]
fn time_points_added_late_keep_distances() {
    let budget = Budget::UNLIMITED;
    let mut inc = IncrementalStn::new();
    let a = inc.add_time_point(&budget).unwrap();
    let b = inc.add_time_point(&budget).unwrap();
    inc.add_constraint(a, b, Some(Ticks(3)), Some(Ticks(4)), &budget)
        .unwrap();
    let c = inc.add_time_point(&budget).unwrap();

    assert_eq!(inc.upper(a, b), Some(Ticks(4)));
    assert_eq!(inc.upper(a, c), None);

    inc.add_constraint(b, c, Some(Ticks(1)), Some(Ticks(1)), &budget)
        .unwrap();
    assert_eq!(inc.upper(a, c), Some(Ticks(5)));
    assert_eq!(inc.lower(a, c), Some(Ticks(4)));
}

/// Budget exhaustion and the memory budget leave the network unchanged.
#[test]
fn budget_errors_leave_network_unchanged() {
    let budget = Budget::UNLIMITED;
    let mut inc = IncrementalStn::new();
    let a = inc.add_time_point(&budget).unwrap();
    let b = inc.add_time_point(&budget).unwrap();
    let tight = Budget {
        max_steps: 3,
        max_cells: 4,
    };

    let result = inc.add_constraint(a, b, Some(Ticks(1)), Some(Ticks(2)), &tight);
    assert_eq!(result, Err(Error::BudgetExhausted));
    assert_eq!(inc.stn().constraints(), []);
    assert_eq!(inc.upper(a, b), None);

    assert_eq!(inc.add_time_point(&tight), Err(Error::MemoryBudgetExceeded));
    assert_eq!(inc.stn().time_point_count(), 2);
}

proptest! {
    /// Every incremental verdict and matrix equals the from-scratch result.
    #[test]
    fn incremental_matches_recompute(spec in any_stn()) {
        replay(&spec)?;
    }

    /// Extreme bounds give a verdict or `Overflow`, never a panic, and overflow changes nothing.
    #[test]
    fn incremental_extreme_values(spec in extreme_stn()) {
        let budget = Budget::UNLIMITED;
        let mut inc = IncrementalStn::new();

        for _ in 0..spec.n {
            inc.add_time_point(&budget).unwrap();
        }

        for &(from, to, lo, hi) in &spec.constraints {
            let before = inc.minimal_network().ok();
            let count = inc.stn().constraints().len();
            let result = inc.add_constraint(
                TimePoint(from), TimePoint(to), lo.map(Ticks), hi.map(Ticks), &budget,
            );

            match result {
                Ok(_) => {}
                Err(Error::Overflow) => {
                    prop_assert_eq!(inc.stn().constraints().len(), count);
                    prop_assert_eq!(inc.minimal_network().ok(), before);
                }
                Err(e) => prop_assert!(false, "unexpected error {e:?}"),
            }
        }
    }
}
