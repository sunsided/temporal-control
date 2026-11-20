//! Property tests: agreement between algorithms, certificate validity, tightness, budgets.

use proptest::prelude::*;
use temporal_control::{
    Bound, Budget, ChordalNetwork, Error, MinimalNetwork, NegativeCycle, Schedule, Stn, Ticks,
    TimePoint, Verdict,
};

use crate::generators::{StnSpec, any_stn, extreme_stn};

/// Whether a verdict says "consistent", ignoring its payload.
pub(crate) fn is_consistent<T>(v: &Verdict<T>) -> bool {
    matches!(v, Verdict::Consistent(_))
}

/// Endpoints and weight of one cycle edge, recomputed from the network's own constraint.
fn edge_of(stn: &Stn, edge: temporal_control::CycleEdge) -> (TimePoint, TimePoint, i128) {
    let c = stn
        .constraint(edge.constraint)
        .expect("cycle names a real constraint");
    match edge.bound {
        Bound::Upper => (
            c.from,
            c.to,
            i128::from(c.hi.expect("upper bound exists").0),
        ),
        Bound::Lower => (
            c.to,
            c.from,
            -i128::from(c.lo.expect("lower bound exists").0),
        ),
    }
}

/// Asserts that `cycle` chains, closes and has the reported negative weight.
pub(crate) fn assert_valid_cycle(stn: &Stn, cycle: &NegativeCycle) {
    let edges: Vec<_> = cycle.edges().iter().map(|&e| edge_of(stn, e)).collect();
    assert_ne!(edges.len(), 0, "cycle must not be empty");

    for (k, &(_, to, _)) in edges.iter().enumerate() {
        let (next_from, _, _) = edges[(k + 1) % edges.len()];
        assert_eq!(to, next_from, "cycle edges must chain and close");
    }

    let weight: i128 = edges.iter().map(|&(_, _, w)| w).sum();
    assert!(weight < 0, "cycle weight {weight} must be negative");
    assert_eq!(weight, i128::from(cycle.weight().0));
}

/// Asserts that `schedule` satisfies every constraint of `stn`.
fn assert_valid_schedule(stn: &Stn, schedule: &Schedule) {
    for c in stn.constraints() {
        let diff = i128::from(schedule.time(c.to).unwrap().0)
            - i128::from(schedule.time(c.from).unwrap().0);
        assert!(c.lo.is_none_or(|lo| i128::from(lo.0) <= diff));
        assert!(c.hi.is_none_or(|hi| diff <= i128::from(hi.0)));
    }
}

/// Whether `spec` plus the extra constraint `lo <= t_j - t_i` is consistent.
fn consistent_with_lower(spec: &StnSpec, i: usize, j: usize, lo: i64) -> bool {
    let mut extended = spec.clone();
    extended.constraints.push((i, j, Some(lo), None));
    let stn = extended.build().unwrap();
    is_consistent(&stn.check_consistency(&Budget::UNLIMITED).unwrap())
}

/// Asserts that every finite bound of `min` is attained and every infinite one is unbounded.
fn assert_tight(spec: &StnSpec, min: &MinimalNetwork) {
    for i in 0..spec.n {
        for j in 0..spec.n {
            match min.upper(TimePoint(i), TimePoint(j)) {
                Some(Ticks(d)) => {
                    assert!(
                        consistent_with_lower(spec, i, j, d),
                        "bound {i}->{j} not attained"
                    );
                    assert!(
                        !consistent_with_lower(spec, i, j, d + 1),
                        "bound {i}->{j} not sound"
                    );
                }
                None => assert!(consistent_with_lower(spec, i, j, 100_000)),
            }
        }
    }
}

/// Asserts that `min` is at least as tight as every original upper and lower bound.
fn assert_within_original(stn: &Stn, min: &MinimalNetwork) {
    for c in stn.constraints() {
        if let Some(hi) = c.hi {
            assert!(min.upper(c.from, c.to).is_some_and(|d| d <= hi));
        }

        if let Some(lo) = c.lo {
            assert!(min.lower(c.from, c.to).is_some_and(|d| d >= lo));
        }
    }
}

/// The mixed generator yields both verdicts often enough for agreement tests to mean something.
#[test]
fn generator_covers_both_verdicts() {
    use proptest::strategy::ValueTree;
    use proptest::test_runner::TestRunner;

    let mut runner = TestRunner::deterministic();
    let strategy = any_stn();
    let samples = 500;
    let consistent = (0..samples)
        .filter(|_| {
            let spec = strategy.new_tree(&mut runner).unwrap().current();
            let stn = spec.build().unwrap();
            is_consistent(&stn.check_consistency(&Budget::UNLIMITED).unwrap())
        })
        .count();

    assert!(
        consistent > samples / 5,
        "only {consistent} of {samples} consistent"
    );
    assert!(
        consistent < samples * 4 / 5,
        "{consistent} of {samples} consistent"
    );
}

/// Asserts that every pair of `net` has the bounds of the minimal network `min`.
fn assert_matches_minimal(net: &ChordalNetwork, min: &MinimalNetwork) {
    for (a, b) in net.pairs() {
        assert!(net.contains(a, b) && net.contains(b, a));
        assert_eq!(net.upper(a, b), min.upper(a, b), "{a:?} -> {b:?}");
        assert_eq!(net.upper(b, a), min.upper(b, a), "{b:?} -> {a:?}");
    }
}

/// `spec` with an unbounded constraint between every pair, so P³C computes all of them.
fn with_all_pairs(spec: &StnSpec) -> StnSpec {
    let mut out = spec.clone();

    for i in 0..spec.n {
        for j in i + 1..spec.n {
            out.constraints.push((i, j, None, None));
        }
    }

    out
}

proptest! {
    /// Bellman-Ford and Floyd-Warshall agree on consistency.
    #[test]
    fn algorithms_agree(spec in any_stn()) {
        let stn = spec.build().unwrap();
        let bf = stn.check_consistency(&Budget::UNLIMITED).unwrap();
        let fw = stn.minimal_network(&Budget::UNLIMITED).unwrap();
        prop_assert_eq!(is_consistent(&bf), is_consistent(&fw));
    }

    /// Every verdict carries a valid certificate: a schedule or a real negative cycle.
    #[test]
    fn certificates_are_valid(spec in any_stn()) {
        let stn = spec.build().unwrap();
        match stn.check_consistency(&Budget::UNLIMITED).unwrap() {
            Verdict::Consistent(schedule) => assert_valid_schedule(&stn, &schedule),
            Verdict::Inconsistent(cycle) => assert_valid_cycle(&stn, &cycle),
        }

        if let Verdict::Inconsistent(cycle) = stn.minimal_network(&Budget::UNLIMITED).unwrap() {
            assert_valid_cycle(&stn, &cycle);
        }
    }

    /// The minimal network is sound and tight.
    #[test]
    fn minimal_network_is_tight(spec in any_stn()) {
        let stn = spec.build().unwrap();
        if let Verdict::Consistent(min) = stn.minimal_network(&Budget::UNLIMITED).unwrap() {
            assert_within_original(&stn, &min);
            assert_tight(&spec, &min);
        }
    }

    /// Under any budget the result is either a budget error or the unlimited result.
    #[test]
    fn budget_never_changes_verdict(
        spec in any_stn(),
        max_steps in 0u64..400,
        max_cells in 0usize..64,
    ) {
        let stn = spec.build().unwrap();
        let budget = Budget { max_steps, max_cells };

        let bf = stn.check_consistency(&budget);
        prop_assert!(bf == Err(Error::BudgetExhausted)
            || bf == stn.check_consistency(&Budget::UNLIMITED));

        let fw = stn.minimal_network(&budget);
        prop_assert!(matches!(fw, Err(Error::BudgetExhausted | Error::MemoryBudgetExceeded))
            || fw == stn.minimal_network(&Budget::UNLIMITED));
    }

    /// Extreme bounds never panic; they produce a verdict or `Overflow`, and verdicts agree.
    #[test]
    fn extreme_values_do_not_panic(spec in extreme_stn()) {
        let Ok(stn) = spec.build() else {
            return Ok(());
        };

        let bf = stn.check_consistency(&Budget::UNLIMITED);
        let fw = stn.minimal_network(&Budget::UNLIMITED);
        prop_assert!(matches!(bf, Ok(_) | Err(Error::Overflow)));
        prop_assert!(matches!(fw, Ok(_) | Err(Error::Overflow)));

        if let (Ok(bf), Ok(fw)) = (&bf, &fw) {
            prop_assert_eq!(is_consistent(bf), is_consistent(fw));
        }

        if let Ok(Verdict::Inconsistent(cycle)) = &bf {
            assert_valid_cycle(&stn, cycle);
        }
    }

    /// P³C agrees with Bellman-Ford on consistency and with Floyd-Warshall on every pair it
    /// computes; with every pair constrained it computes the whole minimal network.
    #[test]
    fn p3c_agrees_with_floyd_warshall(spec in any_stn()) {
        let stn = spec.build().unwrap();
        let bf = stn.check_consistency(&Budget::UNLIMITED).unwrap();
        let ppc = stn.partial_path_consistency(&Budget::UNLIMITED).unwrap();
        prop_assert_eq!(is_consistent(&bf), is_consistent(&ppc));

        if let Verdict::Inconsistent(cycle) = &ppc {
            assert_valid_cycle(&stn, cycle);
        }

        let full = with_all_pairs(&spec).build().unwrap();

        if let (Verdict::Consistent(net), Verdict::Consistent(min), Verdict::Consistent(all)) = (
            ppc,
            stn.minimal_network(&Budget::UNLIMITED).unwrap(),
            full.partial_path_consistency(&Budget::UNLIMITED).unwrap(),
        ) {
            assert_matches_minimal(&net, &min);
            assert_matches_minimal(&all, &min);
            let n = spec.n * spec.n.saturating_sub(1) / 2;
            prop_assert_eq!(all.pairs().count(), n);
        }
    }

    /// Under any budget P³C returns a budget error or the unlimited result; extreme bounds
    /// give a verdict that agrees with Bellman-Ford, or `Overflow`.
    #[test]
    fn p3c_budgets_and_extremes(
        spec in any_stn(),
        extreme in extreme_stn(),
        max_steps in 0u64..400,
        max_cells in 0usize..64,
    ) {
        let stn = spec.build().unwrap();
        let ppc = stn.partial_path_consistency(&Budget { max_steps, max_cells });
        prop_assert!(matches!(ppc, Err(Error::BudgetExhausted | Error::MemoryBudgetExceeded))
            || ppc == stn.partial_path_consistency(&Budget::UNLIMITED));

        if let Ok(stn) = extreme.build() {
            let ppc = stn.partial_path_consistency(&Budget::UNLIMITED);
            prop_assert!(matches!(ppc, Ok(_) | Err(Error::Overflow)));

            if let (Ok(ppc), Ok(bf)) = (&ppc, stn.check_consistency(&Budget::UNLIMITED)) {
                prop_assert_eq!(is_consistent(ppc), is_consistent(&bf));
            }
        }
    }
}
