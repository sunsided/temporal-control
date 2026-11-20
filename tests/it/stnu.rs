//! Dynamic controllability: textbook examples, oracle cross-checks and simulated dispatch.

use proptest::prelude::*;
use temporal_control::{
    Bound, Budget, ContingentId, Controllability, DcAlgorithm, DispatchAlgorithm,
    DispatchableNetwork, Error, LabelledEdge, SemiReducibleCycle, Stnu, Ticks, TimePoint,
};

use crate::oracle::{self, Reaction};
use crate::sim::{self, Agent, Nature, Rng};
use crate::stnu_gen::{StnuSpec, medium_stnu, small_stnu};

/// Budget for checks on generated networks: generous, but a hang fails the test instead.
const TEST_BUDGET: Budget = Budget {
    max_steps: 50_000_000,
    max_cells: 1 << 20,
};

/// The default extended dispatchable network, or `None` if not DC. Panics on errors.
fn estnu(stnu: &Stnu) -> Option<DispatchableNetwork> {
    estnu_with(stnu, DispatchAlgorithm::default())
}

/// The extended dispatchable network of `algorithm`, or `None` if not DC. Panics on errors.
fn estnu_with(stnu: &Stnu, algorithm: DispatchAlgorithm) -> Option<DispatchableNetwork> {
    match stnu
        .dispatchable_with(algorithm, &TEST_BUDGET)
        .unwrap_or_else(|e| panic!("{algorithm:?}: {e}"))
    {
        Controllability::Controllable(net) => Some(net),
        Controllability::Uncontrollable(_) => None,
    }
}

/// Every algorithm behind [`Stnu::dispatchable_with`].
const DISPATCHERS: [DispatchAlgorithm; 3] = [
    DispatchAlgorithm::Minimal,
    DispatchAlgorithm::Morris2014,
    DispatchAlgorithm::Closure,
];

/// Every checker behind [`Stnu::check_controllability_with`].
const ALGORITHMS: [DcAlgorithm; 3] = [
    DcAlgorithm::Morris2014,
    DcAlgorithm::RulMinus,
    DcAlgorithm::PerLabelBackprop,
];

/// Verdict of `algorithm`. Panics on budget exhaustion or an internal disagreement.
fn is_dc(stnu: &Stnu, algorithm: DcAlgorithm) -> bool {
    let verdict = stnu.check_controllability_with(algorithm, &TEST_BUDGET);
    matches!(
        verdict.unwrap_or_else(|e| panic!("{algorithm:?}: {e}")),
        Controllability::Controllable(())
    )
}

/// The explanation of [`Stnu::check_controllability`], or `None` if DC.
fn explanation(stnu: &Stnu) -> Option<SemiReducibleCycle> {
    match stnu
        .check_controllability(&TEST_BUDGET)
        .expect("Morris fits the test budget")
    {
        Controllability::Controllable(()) => None,
        Controllability::Uncontrollable(cycle) => Some(cycle),
    }
}

/// Asserts that `cycle` chains, closes and has the stated negative weight in `stnu`.
fn assert_cycle_closes(stnu: &Stnu, cycle: &SemiReducibleCycle) {
    let edges: Vec<_> = cycle
        .edges()
        .iter()
        .map(|e| e.resolve(stnu).expect("edge belongs to the network"))
        .collect();
    assert_ne!(edges.len(), 0, "empty cycle");

    for (i, &(_, head, _)) in edges.iter().enumerate() {
        let (tail, _, _) = edges[(i + 1) % edges.len()];
        assert_eq!(head, tail, "edge {i} does not chain in {cycle:?}");
    }

    let weight: i64 = edges.iter().map(|e| e.2.0).sum();
    assert_eq!(Ticks(weight), cycle.weight());
    assert!(weight < 0, "{cycle:?}");
}

/// `stnu` with all time points and contingent links, but only the requirement bounds that
/// `cycle` names.
fn restrict(stnu: &Stnu, cycle: &SemiReducibleCycle) -> Stnu {
    let mut out = Stnu::new();

    for p in 0..stnu.time_point_count() {
        match stnu.links().iter().find(|l| l.contingent.0 == p) {
            Some(link) => out.add_contingent_link(link.activation, link.lo, link.hi),
            None => out.add_time_point(),
        }
        .unwrap();
    }

    for edge in cycle.edges() {
        if let LabelledEdge::Requirement(e) = edge {
            let c = stnu.requirements().constraint(e.constraint).unwrap();
            let (lo, hi) = match e.bound {
                Bound::Upper => (None, c.hi),
                Bound::Lower => (c.lo, None),
            };
            out.add_constraint(c.from, c.to, lo, hi).unwrap();
        }
    }

    out
}

/// Contingent `A => C` in `[1, 10]` and executable `X` with `C - X` in `[lo, hi]` plus an
/// optional deadline `X - A <= deadline`.
fn wait_example(lo: i64, hi: i64, deadline: Option<i64>) -> Stnu {
    let mut stnu = Stnu::new();
    let a = stnu.add_time_point().unwrap();
    let c = stnu.add_contingent_link(a, Ticks(1), Ticks(10)).unwrap();
    let x = stnu.add_time_point().unwrap();
    stnu.add_constraint(x, c, Some(Ticks(lo)), Some(Ticks(hi)))
        .unwrap();
    stnu.add_constraint(a, x, Some(Ticks(0)), deadline.map(Ticks))
        .unwrap();
    stnu
}

/// `X` must precede `C` by 1 to 2 ticks without knowing `C`: not DC.
#[test]
fn must_precede_unknown_end_is_not_dc() {
    let stnu = wait_example(1, 2, None);
    assert!(estnu(&stnu).is_none());

    // The cycle needs the maximum duration of the link: X cannot wait for C.
    let cycle = explanation(&stnu).expect("not DC");
    assert_cycle_closes(&stnu, &cycle);
    assert!(
        cycle
            .edges()
            .contains(&LabelledEdge::UpperCase(ContingentId(0)))
    );
}

/// `X` within 2 ticks before or after `C` and `X <= A + 8`: DC by waiting for `C` until `A + 8`.
#[test]
fn wait_until_deadline_is_dc() {
    let stnu = wait_example(-10, 2, Some(8));

    for algorithm in DISPATCHERS {
        let net = estnu_with(&stnu, algorithm).expect("DC");
        let wait = net.waits().iter().find(|w| w.waiter == TimePoint(2));
        assert_eq!(wait.map(|w| w.delay), Some(Ticks(8)), "{algorithm:?}");

        for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
            for nature in [Nature::Min, Nature::Max, Nature::Random] {
                let times = sim::run(&net, agent, nature, &mut Rng(11));
                assert_satisfies(&stnu, &times);
            }
        }
    }
}

/// With deadline `A + 7` the agent must commit before it can be sure: not DC.
#[test]
fn wait_past_deadline_is_not_dc() {
    assert!(estnu(&wait_example(-10, 2, Some(7))).is_none());
}

/// `X` exactly at `C` is DC only with same-tick reaction, which these semantics allow.
#[test]
fn same_tick_reaction_is_dc() {
    assert!(estnu(&wait_example(0, 0, None)).is_some());
}

/// The rover: warm-up `[5, 15]`, analysis starts at most 5 ticks after warm-up ends and takes
/// `[10, 25]`, transmit after analysis by a deadline from the start. DC iff deadline >= 40.
fn rover(deadline: i64) -> Stnu {
    let mut stnu = Stnu::new();
    let start = stnu.add_time_point().unwrap();
    let warm = stnu
        .add_contingent_link(start, Ticks(5), Ticks(15))
        .unwrap();
    let analysis = stnu.add_time_point().unwrap();
    let done = stnu
        .add_contingent_link(analysis, Ticks(10), Ticks(25))
        .unwrap();
    let transmit = stnu.add_time_point().unwrap();
    stnu.add_constraint(warm, analysis, Some(Ticks(0)), Some(Ticks(5)))
        .unwrap();
    stnu.add_constraint(done, transmit, Some(Ticks(0)), None)
        .unwrap();
    stnu.add_constraint(start, transmit, Some(Ticks(0)), Some(Ticks(deadline)))
        .unwrap();
    stnu
}

/// The rover deadline threshold, and dispatch under extreme durations.
#[test]
fn rover_deadline_threshold() {
    let late = rover(39);
    assert!(estnu(&late).is_none());
    let cycle = explanation(&late).expect("not DC");
    assert_cycle_closes(&late, &cycle);
    assert_eq!(cycle.weight(), Ticks(-1), "one tick short");
    assert!(estnu(&restrict(&late, &cycle)).is_none());

    let stnu = rover(40);
    let net = estnu(&stnu).expect("DC");

    for nature in [Nature::Min, Nature::Max, Nature::Random] {
        let times = sim::run(&net, Agent::Random, nature, &mut Rng(3));
        assert_satisfies(&stnu, &times);
    }
}

/// Without contingent links, DC equals STN consistency.
#[test]
fn no_links_matches_stn() {
    let mut stnu = Stnu::new();
    let a = stnu.add_time_point().unwrap();
    let b = stnu.add_time_point().unwrap();
    stnu.add_constraint(a, b, Some(Ticks(3)), Some(Ticks(2)))
        .unwrap();
    assert!(estnu(&stnu).is_none());
}

/// `X1` and `X2` follow `C` (`Z => C` in `[1, 7]`) closely and must happen by `Z + 6`. The
/// paper's diamond stand-in `X1 -> X2: 0` would let the minimisation drop `X2`'s deadline,
/// which this dispatcher (no propagation along waits) then never learns.
#[test]
fn minimal_keeps_deadlines_behind_waits() {
    let mut stnu = Stnu::new();
    let z = stnu.add_time_point().unwrap();
    let x1 = stnu.add_time_point().unwrap();
    let x2 = stnu.add_time_point().unwrap();
    let c = stnu.add_contingent_link(z, Ticks(1), Ticks(7)).unwrap();
    stnu.add_constraint(x1, c, Some(Ticks(-1)), Some(Ticks(1)))
        .unwrap();
    stnu.add_constraint(x2, c, Some(Ticks(0)), Some(Ticks(1)))
        .unwrap();

    for x in [x1, x2] {
        stnu.add_constraint(z, x, Some(Ticks(0)), Some(Ticks(6)))
            .unwrap();
    }

    let net = estnu(&stnu).expect("DC");

    for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
        for nature in [Nature::Min, Nature::Max, Nature::Random] {
            for seed in 1..20 {
                assert_satisfies(&stnu, &sim::run(&net, agent, nature, &mut Rng(seed)));
            }
        }
    }
}

/// A chain `Z => C1 => C2`, each `[1, 2]`, with `C2 <= Z`: not DC. RUL⁻ needs its activation
/// points normalised to executable ones to see this.
#[test]
fn chained_links_are_checked() {
    let mut stnu = Stnu::new();
    let z = stnu.add_time_point().unwrap();
    let c1 = stnu.add_contingent_link(z, Ticks(1), Ticks(2)).unwrap();
    let c2 = stnu.add_contingent_link(c1, Ticks(1), Ticks(2)).unwrap();
    stnu.add_constraint(c2, z, Some(Ticks(0)), None).unwrap();

    for algorithm in ALGORITHMS {
        assert!(!is_dc(&stnu, algorithm), "{algorithm:?}");
    }

    let cycle = explanation(&stnu).expect("not DC");
    assert_cycle_closes(&stnu, &cycle);
}

/// Invalid contingent bounds are rejected.
#[test]
fn invalid_links_rejected() {
    let mut stnu = Stnu::new();
    let a = stnu.add_time_point().unwrap();
    assert_eq!(
        stnu.add_contingent_link(a, Ticks(0), Ticks(3)),
        Err(Error::InvalidContingentLink)
    );
    assert_eq!(
        stnu.add_contingent_link(a, Ticks(4), Ticks(3)),
        Err(Error::InvalidContingentLink)
    );
    assert_eq!(
        stnu.add_contingent_link(TimePoint(9), Ticks(1), Ticks(3)),
        Err(Error::UnknownTimePoint)
    );
}

/// Asserts that `times` satisfies every requirement and contingent bound of `stnu`.
fn assert_satisfies(stnu: &Stnu, times: &[Ticks]) {
    sim::assert_satisfies(stnu.requirements(), times);

    for link in stnu.links() {
        let d = times[link.contingent.0].0 - times[link.activation.0].0;
        assert!(
            link.lo.0 <= d && d <= link.hi.0,
            "{link:?} broken by {times:?}"
        );
    }
}

/// Counts `(DC, DC with waits)` among `samples` networks drawn from `strategy`.
fn coverage(strategy: &impl Strategy<Value = StnuSpec>, samples: usize) -> (usize, usize) {
    use proptest::strategy::ValueTree;
    use proptest::test_runner::TestRunner;

    let mut runner = TestRunner::deterministic();
    let (mut dc, mut with_waits) = (0, 0);

    for _ in 0..samples {
        let spec = strategy.new_tree(&mut runner).unwrap().current();

        // The closure keeps every wait, so it measures how often waits are needed at all.
        if let Some(net) = estnu_with(&spec.build(), DispatchAlgorithm::Closure) {
            dc += 1;
            with_waits += usize::from(!net.waits().is_empty());
        }
    }

    (dc, with_waits)
}

/// Both generators yield both verdicts, and DC networks with waits, often enough.
#[test]
fn generators_cover_verdicts_and_waits() {
    let samples = 400;

    for (name, (dc, waits)) in [
        ("small", coverage(&small_stnu(), samples)),
        ("medium", coverage(&medium_stnu(), samples)),
    ] {
        assert!(dc > samples / 5 && dc < samples * 4 / 5, "{name}: {dc} DC");
        assert!(waits > samples / 20, "{name}: {waits} with waits");
    }
}

/// Total edges plus waits of every algorithm's network over the DC samples of `strategy`.
fn sizes(strategy: &impl Strategy<Value = StnuSpec>, samples: usize) -> [usize; 3] {
    use proptest::strategy::ValueTree;
    use proptest::test_runner::TestRunner;

    let mut runner = TestRunner::deterministic();
    let mut total = [0; 3];

    for _ in 0..samples {
        let stnu = strategy.new_tree(&mut runner).unwrap().current().build();

        for (i, algorithm) in DISPATCHERS.into_iter().enumerate() {
            if let Some(net) = estnu_with(&stnu, algorithm) {
                total[i] += net.edges().len() + net.waits().len();
            }
        }
    }

    total
}

/// The minimised network is smaller than Morris's, which is smaller than the closure's, in
/// total over random networks (not necessarily for each one).
#[test]
fn minimisation_shrinks_networks() {
    for strategy in [small_stnu().boxed(), medium_stnu().boxed()] {
        let [minimal, morris, closure] = sizes(&strategy, 400);
        println!("edges + waits: minimal {minimal}, Morris {morris}, closure {closure}");
        assert!(minimal < morris && morris < closure);
    }
}

/// Runs the dispatcher on `spec` under every policy pair.
fn simulate_all(spec: &StnuSpec, net: &DispatchableNetwork, seed: u64) {
    let stnu = spec.build();
    let mut rng = Rng(seed);

    for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
        for nature in [Nature::Min, Nature::Max, Nature::Random] {
            let times = sim::run(net, agent, nature, &mut rng);
            assert_satisfies(&stnu, &times);
        }
    }
}

proptest! {
    /// Every checker agrees with the brute-force game under same-tick reaction.
    #[test]
    fn checkers_match_oracle(spec in small_stnu()) {
        let expected = oracle::is_dc(&spec, Reaction::Instant);
        let stnu = spec.build();

        for algorithm in DISPATCHERS {
            prop_assert_eq!(estnu_with(&stnu, algorithm).is_some(), expected, "{:?}", algorithm);
        }

        for algorithm in ALGORITHMS {
            prop_assert_eq!(is_dc(&stnu, algorithm), expected, "{:?}", algorithm);
        }
    }

    /// On networks too large for the oracle, every checker still agrees with the closure, and
    /// the closure's network dispatches without violations.
    #[test]
    fn checkers_agree_on_larger_networks(spec in medium_stnu(), seed in 1u64..) {
        let stnu = spec.build();
        let expected = estnu_with(&stnu, DispatchAlgorithm::Closure).is_some();

        for algorithm in ALGORITHMS {
            prop_assert_eq!(expected, is_dc(&stnu, algorithm), "{:?}", algorithm);
        }

        for algorithm in DISPATCHERS {
            let net = estnu_with(&stnu, algorithm);
            prop_assert_eq!(net.is_some(), expected, "{:?}", algorithm);

            if let Some(net) = net {
                simulate_all(&spec, &net, seed);
            }
        }
    }

    /// Next-tick reaction is stricter: whatever it can control, same-tick reaction can too.
    #[test]
    fn next_tick_is_stricter(spec in small_stnu()) {
        if oracle::is_dc(&spec, Reaction::NextTick) {
            prop_assert!(oracle::is_dc(&spec, Reaction::Instant));
        }
    }

    /// Dispatching a DC network never violates a constraint, whatever nature and the agent do.
    #[test]
    fn dispatch_never_violates(spec in small_stnu(), seed in 1u64..) {
        for algorithm in DISPATCHERS {
            if let Some(net) = estnu_with(&spec.build(), algorithm) {
                simulate_all(&spec, &net, seed);
            }
        }
    }

    /// Under any budget the result is a budget error or the unlimited result.
    #[test]
    fn budget_never_changes_verdict(spec in small_stnu(), max_steps in 0u64..5_000) {
        let stnu = spec.build();
        let budget = Budget { max_steps, max_cells: usize::MAX };
        for algorithm in DISPATCHERS {
            let limited = stnu.dispatchable_with(algorithm, &budget);
            prop_assert!(limited == Err(Error::BudgetExhausted)
                || limited == stnu.dispatchable_with(algorithm, &TEST_BUDGET));
        }

        for algorithm in ALGORITHMS {
            let limited = stnu.check_controllability_with(algorithm, &budget);
            prop_assert!(limited == Err(Error::BudgetExhausted)
                || limited == stnu.check_controllability_with(algorithm, &TEST_BUDGET));
        }
    }

    /// A small network's explanation closes, and its requirement bounds alone (with all links
    /// and the oracle's horizon) are already not DC for the game.
    #[test]
    fn explanation_is_certificate_for_oracle(spec in small_stnu()) {
        if let Some(cycle) = explanation(&spec.build()) {
            assert_cycle_closes(&spec.build(), &cycle);
            prop_assert!(!oracle::is_dc(&restrict_spec(&spec, &cycle), Reaction::Instant));
        }
    }

    /// On larger networks, the cited requirement bounds alone are not DC for the closure.
    #[test]
    fn explanation_is_certificate(spec in medium_stnu()) {
        let stnu = spec.build();

        if let Some(cycle) = explanation(&stnu) {
            assert_cycle_closes(&stnu, &cycle);
            prop_assert!(estnu(&restrict(&stnu, &cycle)).is_none());
        }
    }
}

/// `spec` with only the requirement bounds that `cycle` names, in the oracle's format.
///
/// The horizon constraints come back with [`StnuSpec::build`]; they only make the
/// network harder to control, so the check stays sound, just slightly weaker.
fn restrict_spec(spec: &StnuSpec, cycle: &SemiReducibleCycle) -> StnuSpec {
    let all: Vec<_> = spec.all_constraints().collect();
    let constraints = cycle
        .edges()
        .iter()
        .filter_map(|edge| match edge {
            LabelledEdge::Requirement(e) => {
                let (u, v, lo, hi) = all[e.constraint.0];
                Some(match e.bound {
                    Bound::Upper => (u, v, None, hi),
                    Bound::Lower => (u, v, lo, None),
                })
            }
            _ => None,
        })
        .collect();

    StnuSpec {
        constraints,
        ..spec.clone()
    }
}
