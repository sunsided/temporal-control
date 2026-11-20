//! DC checkers on arbitrary small STNUs: no panic, and every checker that answers agrees.
//!
//! Runs every [`DcAlgorithm`] through `Stnu::check_controllability_with` and the closure
//! through `Stnu::dispatchable`, under a finite budget (the closure has no proven round
//! bound). Budget errors are skipped; `Error::Internal` is a failure. A DC verdict's
//! dispatchable network must load into a dispatcher and survive an encoding round trip.

#![no_main]

use libfuzzer_sys::arbitrary::{self, Arbitrary};
use libfuzzer_sys::fuzz_target;
use temporal_control::{
    Budget, Controllability, DcAlgorithm, DispatchableNetwork, Dispatcher, Error, Stnu, Ticks,
    TimePoint,
};

/// Largest number of time points a fuzzed STNU gets.
const MAX_POINTS: usize = 8;

/// Budget per check: large enough for every network of [`MAX_POINTS`] points in practice.
const BUDGET: Budget = Budget {
    max_steps: 2_000_000,
    max_cells: 1 << 12,
};

/// Every checker behind `check_controllability_with`.
const ALGORITHMS: [DcAlgorithm; 3] = [
    DcAlgorithm::Morris2014,
    DcAlgorithm::RulMinus,
    DcAlgorithm::PerLabelBackprop,
];

/// One construction step; indices are reduced modulo the current point count.
#[derive(Debug, Clone, Copy, Arbitrary)]
enum Op {
    /// `add_time_point`.
    Point,
    /// `add_contingent_link(activation, lo, lo + extra)`.
    Link(u8, i8, u8),
    /// `add_constraint(from, to, lo, hi)`.
    Constraint(u8, u8, Option<i8>, Option<i8>),
}

fuzz_target!(|ops: Vec<Op>| {
    let Some(stnu) = build(&ops) else {
        return;
    };

    let mut verdicts: Vec<(String, bool)> = Vec::new();

    for algorithm in ALGORITHMS {
        let verdict = stnu.check_controllability_with(algorithm, &BUDGET);

        if let Some(dc) = answer(&verdict.map(|c| matches!(c, Controllability::Controllable(())))) {
            verdicts.push((format!("{algorithm:?}"), dc));
        }
    }

    let closure = stnu.dispatchable(&BUDGET);

    if let Ok(Controllability::Controllable(net)) = &closure {
        check_network(net);
    }

    if let Some(dc) = answer(&closure.map(|c| matches!(c, Controllability::Controllable(_)))) {
        verdicts.push(("closure".into(), dc));
    }

    assert!(
        verdicts.windows(2).all(|w| w[0].1 == w[1].1),
        "checkers disagree: {verdicts:?} on {stnu:?}"
    );
});

/// Builds the STNU, or `None` if it would be too large or a step is rejected.
fn build(ops: &[Op]) -> Option<Stnu> {
    let mut stnu = Stnu::new();
    stnu.add_time_point().ok()?;

    for &op in ops {
        let n = stnu.time_point_count();
        let tp = |i: u8| TimePoint(usize::from(i) % n);
        let grows = matches!(op, Op::Point | Op::Link(..));

        if grows && n >= MAX_POINTS {
            continue;
        }

        match op {
            Op::Point => drop(stnu.add_time_point().ok()?),
            Op::Link(a, lo, extra) => {
                let lo = i64::from(lo);
                let hi = lo + i64::from(extra % 16);
                // Rejected links (lo <= 0) are part of the input space; skip them.
                let _ = stnu.add_contingent_link(tp(a), Ticks(lo), Ticks(hi));
            }
            Op::Constraint(u, v, lo, hi) => {
                let bound = |b: Option<i8>| b.map(|x| Ticks(i64::from(x)));
                stnu.add_constraint(tp(u), tp(v), bound(lo), bound(hi))
                    .ok()?;
            }
        }
    }

    Some(stnu)
}

/// The verdict if the checker answered, `None` on budget exhaustion; panics on
/// `Error::Internal` and on errors a small STNU must not produce.
fn answer(result: &Result<bool, Error>) -> Option<bool> {
    match result {
        Ok(dc) => Some(*dc),
        Err(Error::BudgetExhausted | Error::MemoryBudgetExceeded) => None,
        Err(e) => panic!("checker failed: {e:?}"),
    }
}

/// A dispatchable network from the closure loads into a dispatcher and round-trips.
fn check_network(net: &DispatchableNetwork) {
    let mut d = Dispatcher::<MAX_POINTS, 1024>::EMPTY;
    d.load(net.points(), net.edges(), net.waits())
        .expect("a DC verdict's network is valid");

    let bytes = net.encode().expect("small");
    assert_eq!(DispatchableNetwork::decode(&bytes).as_ref(), Ok(net));
    d.load_encoded(&bytes).expect("its encoding loads");
}
