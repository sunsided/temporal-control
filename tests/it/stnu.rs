//! Dynamic controllability: textbook examples, oracle cross-checks and simulated dispatch.

use proptest::prelude::*;
use temporal_control::{
    Budget, Controllability, DispatchableNetwork, Error, Stnu, Ticks, TimePoint,
};

use crate::oracle::{self, Reaction};
use crate::sim::{self, Agent, Nature, Rng};
use crate::stnu_gen::{StnuSpec, medium_stnu, small_stnu};

/// Budget for checks on generated networks: generous, but a hang fails the test instead.
const TEST_BUDGET: Budget = Budget {
    max_steps: 50_000_000,
    max_cells: 1 << 20,
};

/// The extended dispatchable network, or `None` if not DC. Panics on budget exhaustion.
fn estnu(stnu: &Stnu) -> Option<DispatchableNetwork> {
    match stnu
        .dispatchable(&TEST_BUDGET)
        .expect("closure fits the test budget")
    {
        Controllability::Controllable(net) => Some(net),
        Controllability::Uncontrollable => None,
    }
}

/// Verdict of [`Stnu::check_controllability`]. Panics on budget exhaustion.
fn is_dc_backprop(stnu: &Stnu) -> bool {
    let verdict = stnu.check_controllability(&TEST_BUDGET);
    matches!(
        verdict.expect("backprop fits the test budget"),
        Controllability::Controllable(())
    )
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
    assert!(estnu(&wait_example(1, 2, None)).is_none());
}

/// `X` within 2 ticks before or after `C` and `X <= A + 8`: DC by waiting for `C` until `A + 8`.
#[test]
fn wait_until_deadline_is_dc() {
    let stnu = wait_example(-10, 2, Some(8));
    let net = estnu(&stnu).expect("DC");
    let wait = net.waits().iter().find(|w| w.waiter == TimePoint(2));
    assert_eq!(wait.map(|w| w.delay), Some(Ticks(8)));

    for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
        for nature in [Nature::Min, Nature::Max, Nature::Random] {
            let times = sim::run(&net, agent, nature, &mut Rng(11));
            assert_satisfies(&stnu, &times);
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
    assert!(estnu(&rover(39)).is_none());
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

        if let Some(net) = estnu(&spec.build()) {
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
    /// Closure and backpropagation agree with the brute-force game under same-tick reaction.
    #[test]
    fn checkers_match_oracle(spec in small_stnu()) {
        let expected = oracle::is_dc(&spec, Reaction::Instant);
        let stnu = spec.build();
        prop_assert_eq!(estnu(&stnu).is_some(), expected, "closure");
        prop_assert_eq!(is_dc_backprop(&stnu), expected, "backprop");
    }

    /// On networks too large for the oracle, closure and backpropagation still agree, and the
    /// closure's network dispatches without violations.
    #[test]
    fn checkers_agree_on_larger_networks(spec in medium_stnu(), seed in 1u64..) {
        let stnu = spec.build();
        let net = estnu(&stnu);
        prop_assert_eq!(net.is_some(), is_dc_backprop(&stnu));

        if let Some(net) = net {
            simulate_all(&spec, &net, seed);
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
        if let Some(net) = estnu(&spec.build()) {
            simulate_all(&spec, &net, seed);
        }
    }

    /// Under any budget the result is a budget error or the unlimited result.
    #[test]
    fn budget_never_changes_verdict(spec in small_stnu(), max_steps in 0u64..5_000) {
        let stnu = spec.build();
        let budget = Budget { max_steps, max_cells: usize::MAX };
        let limited = stnu.dispatchable(&budget);
        prop_assert!(limited == Err(Error::BudgetExhausted)
            || limited == stnu.dispatchable(&TEST_BUDGET));

        let limited = stnu.check_controllability(&budget);
        prop_assert!(limited == Err(Error::BudgetExhausted)
            || limited == stnu.check_controllability(&TEST_BUDGET));
    }
}
