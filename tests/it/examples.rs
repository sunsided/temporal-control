//! Hand-written example networks with known answers.

use temporal_control::{Bound, Budget, Error, Stn, Ticks, TimePoint, Verdict};

/// Builds the rover example: warm-up 5..15 before analysis, analysis 10..25 before transmit,
/// and a deadline on transmit relative to warm-up.
fn rover(deadline: i64) -> (Stn, [TimePoint; 3]) {
    let mut stn = Stn::new();
    let warmup = stn.add_time_point();
    let analysis = stn.add_time_point();
    let transmit = stn.add_time_point();
    stn.add_constraint(warmup, analysis, Some(Ticks(5)), Some(Ticks(15)))
        .unwrap();
    stn.add_constraint(analysis, transmit, Some(Ticks(10)), Some(Ticks(25)))
        .unwrap();
    stn.add_constraint(warmup, transmit, None, Some(Ticks(deadline)))
        .unwrap();
    (stn, [warmup, analysis, transmit])
}

/// A feasible deadline yields a schedule and a tightened minimal network.
#[test]
fn rover_consistent_minimal_network() {
    let (stn, [w, a, t]) = rover(20);

    let Verdict::Consistent(min) = stn.minimal_network(&Budget::UNLIMITED).unwrap() else {
        panic!("expected consistent");
    };

    assert_eq!(min.upper(w, a), Some(Ticks(10)));
    assert_eq!(min.lower(w, a), Some(Ticks(5)));
    assert_eq!(min.upper(w, t), Some(Ticks(20)));
    assert_eq!(min.lower(w, t), Some(Ticks(15)));
}

/// A deadline shorter than the minimum durations is explained by exactly the three constraints.
#[test]
fn rover_inconsistent_explained() {
    let (stn, _) = rover(14);

    let Verdict::Inconsistent(cycle) = stn.check_consistency(&Budget::UNLIMITED).unwrap() else {
        panic!("expected inconsistent");
    };

    assert_eq!(cycle.weight(), Ticks(-1));
    assert_eq!(cycle.edges().len(), 3);
    let lower = cycle.edges().iter().filter(|e| e.bound == Bound::Lower);
    assert_eq!(lower.count(), 2);
}

/// A lower bound of `i64::MIN` cannot be negated into a distance-graph edge.
#[test]
fn min_lower_bound_rejected() {
    let mut stn = Stn::new();
    let a = stn.add_time_point();
    let b = stn.add_time_point();
    let result = stn.add_constraint(a, b, Some(Ticks(i64::MIN)), None);
    assert_eq!(result, Err(Error::Overflow));
}

/// Unknown handles are rejected at construction.
#[test]
fn unknown_time_point_rejected() {
    let mut stn = Stn::new();
    let a = stn.add_time_point();
    let result = stn.add_constraint(a, TimePoint(7), None, None);
    assert_eq!(result, Err(Error::UnknownTimePoint));
}

/// A memory budget below `n * n` cells refuses Floyd-Warshall up front.
#[test]
fn memory_budget_enforced() {
    let (stn, _) = rover(20);
    let budget = Budget {
        max_steps: u64::MAX,
        max_cells: 8,
    };
    assert_eq!(
        stn.minimal_network(&budget),
        Err(Error::MemoryBudgetExceeded)
    );
}
