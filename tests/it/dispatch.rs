//! Dispatchable STNs: equivalence of the filtered network and simulated dispatch.

use proptest::prelude::*;
use temporal_control::{
    Budget, DispatchEdge, DispatchError, DispatchableNetwork, Dispatcher, PointKind, Stn, Ticks,
    TimePoint, Verdict, Wait,
};

use crate::generators::any_stn;
use crate::sim::{self, Agent, Nature, Rng};

/// Builds an [`Stn`] from the edges of a dispatchable network.
fn stn_of(net: &DispatchableNetwork) -> Stn {
    let mut stn = Stn::new();

    for _ in net.points() {
        stn.add_time_point().unwrap();
    }

    for e in net.edges() {
        stn.add_constraint(e.from, e.to, None, Some(e.weight))
            .unwrap();
    }

    stn
}

/// The dispatchable network of a consistent STN, or `None`.
fn dispatchable(stn: &Stn) -> Option<DispatchableNetwork> {
    match stn.dispatchable(&Budget::UNLIMITED).unwrap() {
        Verdict::Consistent(net) => Some(net),
        Verdict::Inconsistent(_) => None,
    }
}

/// A rigid pair whose lower index is the later point, plus a third point.
///
/// `t_0 = t_1 + 3`, `t_2 - t_1` in `[1, 10]`, and `t_2 = t_3` (zero-weight edges).
fn rigid_example() -> Stn {
    let mut stn = Stn::new();
    let p: Vec<TimePoint> = (0..4).map(|_| stn.add_time_point().unwrap()).collect();
    stn.add_constraint(p[1], p[0], Some(Ticks(3)), Some(Ticks(3)))
        .unwrap();
    stn.add_constraint(p[1], p[2], Some(Ticks(1)), Some(Ticks(10)))
        .unwrap();
    stn.add_constraint(p[2], p[3], Some(Ticks(0)), Some(Ticks(0)))
        .unwrap();
    stn
}

/// The earliest member leads a rigid component even if it has the higher index.
#[test]
fn rigid_component_led_by_earliest_member() {
    let stn = rigid_example();
    let net = dispatchable(&stn).unwrap();

    assert!(net.edges().contains(&DispatchEdge {
        from: TimePoint(1),
        to: TimePoint(0),
        weight: Ticks(3),
    }));
    assert!(net.edges().iter().all(|e| e.from.0 != 0 || e.to.0 == 1));

    for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
        for nature in [Nature::Min, Nature::Max] {
            let times = sim::run(&net, agent, nature, &mut Rng(7));
            sim::assert_satisfies(&stn, &times);
        }
    }
}

/// Filtering removes edges implied by others: a chain keeps only its links.
#[test]
fn chain_is_filtered_to_its_links() {
    let mut stn = Stn::new();
    let p: Vec<TimePoint> = (0..4).map(|_| stn.add_time_point().unwrap()).collect();

    for w in p.windows(2) {
        stn.add_constraint(w[0], w[1], Some(Ticks(2)), Some(Ticks(5)))
            .unwrap();
    }

    let net = dispatchable(&stn).unwrap();
    assert_eq!(net.edges().len(), 6);
}

/// The dispatcher refuses early execution, reports missed deadlines and rejects bad input.
#[test]
fn dispatcher_reports_errors() {
    let points = [PointKind::Executable; 2];
    let edges = [
        DispatchEdge {
            from: TimePoint(0),
            to: TimePoint(1),
            weight: Ticks(10),
        },
        DispatchEdge {
            from: TimePoint(1),
            to: TimePoint(0),
            weight: Ticks(-5),
        },
    ];
    let mut d = Dispatcher::<2, 4>::new(&points, &edges, &[]).unwrap();

    assert_eq!(
        d.execute(TimePoint(1), Ticks(0)),
        Err(DispatchError::NotExecutable(TimePoint(1)))
    );
    d.execute(TimePoint(0), Ticks(0)).unwrap();
    assert_eq!(
        d.window(TimePoint(1)),
        Some((Some(Ticks(5)), Some(Ticks(10))))
    );
    assert_eq!(d.next_wakeup(), Some(Ticks(5)));
    assert_eq!(
        d.check_deadlines(Ticks(11)),
        Err(DispatchError::DeadlineMissed(TimePoint(1)))
    );
    assert_eq!(
        d.check_deadlines(Ticks(3)),
        Err(DispatchError::TimeWentBackwards)
    );

    assert_eq!(
        Dispatcher::<1, 4>::new(&points, &edges, &[]).err(),
        Some(DispatchError::CapacityExceeded)
    );
    let bad = [DispatchEdge {
        from: TimePoint(0),
        to: TimePoint(5),
        weight: Ticks(1),
    }];
    assert_eq!(
        Dispatcher::<2, 4>::new(&points, &bad, &[]).err(),
        Some(DispatchError::InvalidNetwork)
    );
}

proptest! {
    /// The filtered network has the same minimal network as the original.
    #[test]
    fn filtered_network_is_equivalent(spec in any_stn()) {
        let stn = spec.build().unwrap();
        let Some(net) = dispatchable(&stn) else {
            return Ok(());
        };

        let original = stn.minimal_network(&Budget::UNLIMITED).unwrap();
        let filtered = stn_of(&net).minimal_network(&Budget::UNLIMITED).unwrap();
        prop_assert_eq!(original, filtered);
        prop_assert!(net.edges().iter().all(|e| e.from != e.to));
    }

    /// Any agent policy completes without a dispatch error and satisfies every constraint.
    #[test]
    fn dispatch_never_violates(spec in any_stn(), seed in 1u64..) {
        let stn = spec.build().unwrap();
        let Some(net) = dispatchable(&stn) else {
            return Ok(());
        };

        let mut rng = Rng(seed);

        for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
            let times = sim::run(&net, agent, Nature::Random, &mut rng);
            sim::assert_satisfies(&stn, &times);
        }
    }

    /// Under any budget, filtering either reports exhaustion or matches the unlimited result.
    #[test]
    fn dispatchable_budget_never_changes_result(spec in any_stn(), max_steps in 0u64..600) {
        let stn = spec.build().unwrap();
        let budget = Budget { max_steps, max_cells: usize::MAX };
        let limited = stn.dispatchable(&budget);
        prop_assert!(limited == Err(temporal_control::Error::BudgetExhausted)
            || limited == stn.dispatchable(&Budget::UNLIMITED));
    }
}

/// Upper dominance through a zero-weight edge: `t_0`'s deadline reaches it only through `t_1`.
///
/// Regression: with zero-weight edges counted as precedence, `t_1` waited for `t_0`, which had
/// no deadline of its own, and `t_1` missed its deadline.
#[test]
fn zero_weight_dominance_does_not_deadlock() {
    let mut stn = Stn::new();
    let p: Vec<TimePoint> = (0..6).map(|_| stn.add_time_point().unwrap()).collect();
    stn.add_constraint(p[0], p[1], Some(Ticks(0)), None)
        .unwrap();
    stn.add_constraint(p[5], p[0], Some(Ticks(54)), None)
        .unwrap();
    stn.add_constraint(p[1], p[2], Some(Ticks(-49)), None)
        .unwrap();
    let net = dispatchable(&stn).unwrap();

    for seed in 1..200 {
        let times = sim::run(&net, Agent::Random, Nature::Random, &mut Rng(seed));
        sim::assert_satisfies(&stn, &times);
    }
}

/// A contingent point with the given activation and bounds.
fn contingent(activation: usize, lo: i64, hi: i64) -> PointKind {
    PointKind::Contingent {
        activation: TimePoint(activation),
        lo: Ticks(lo),
        hi: Ticks(hi),
    }
}

/// Every malformed-network branch of the validator is rejected.
#[test]
fn validator_rejects_malformed_networks() {
    let exec = PointKind::Executable;
    let edge = |from, to, weight| DispatchEdge {
        from: TimePoint(from),
        to: TimePoint(to),
        weight: Ticks(weight),
    };
    let wait = |waiter, contingent| Wait {
        waiter: TimePoint(waiter),
        contingent: TimePoint(contingent),
        delay: Ticks(1),
    };
    let cases: [(&[PointKind], &[DispatchEdge], &[Wait]); 7] = [
        (&[exec, contingent(0, 0, 3)], &[], &[]),
        (&[exec, contingent(0, 4, 3)], &[], &[]),
        (&[contingent(1, 1, 3), contingent(0, 1, 3)], &[], &[]),
        (&[exec, contingent(7, 1, 3)], &[], &[]),
        (&[exec, exec], &[edge(1, 1, -1)], &[]),
        (&[exec, contingent(0, 1, 3)], &[], &[wait(1, 1)]),
        (&[exec, exec], &[], &[wait(0, 1)]),
    ];

    for (points, edges, waits) in cases {
        assert_eq!(
            Dispatcher::<4, 8>::new(points, edges, waits).err(),
            Some(DispatchError::InvalidNetwork),
            "{points:?} {edges:?} {waits:?}"
        );
    }
}

/// One arbitrary dispatcher call.
#[derive(Debug, Clone)]
enum Call {
    /// [`Dispatcher::observe`].
    Observe(usize, i64),
    /// [`Dispatcher::execute`].
    Execute(usize, i64),
    /// [`Dispatcher::step`] with one observation.
    Step(usize, i64),
    /// [`Dispatcher::check_deadlines`].
    Deadlines(i64),
}

/// Extreme and small tick values.
fn any_tick() -> impl Strategy<Value = i64> {
    prop_oneof![
        Just(i64::MIN),
        Just(i64::MAX),
        Just(i64::MIN + 1),
        Just(i64::MAX - 1),
        -20i64..=20,
    ]
}

proptest! {
    /// Arbitrary, possibly malformed networks and call sequences never panic.
    #[test]
    fn dispatcher_never_panics(
        kinds in proptest::collection::vec((any::<bool>(), 0usize..6, any_tick(), any_tick()), 0..6),
        edges in proptest::collection::vec((0usize..7, 0usize..7, any_tick()), 0..10),
        waits in proptest::collection::vec((0usize..7, 0usize..7, any_tick()), 0..4),
        calls in proptest::collection::vec(
            prop_oneof![
                (0usize..7, any_tick()).prop_map(|(p, t)| Call::Observe(p, t)),
                (0usize..7, any_tick()).prop_map(|(p, t)| Call::Execute(p, t)),
                (0usize..7, any_tick()).prop_map(|(p, t)| Call::Step(p, t)),
                any_tick().prop_map(Call::Deadlines),
            ],
            0..20,
        ),
    ) {
        let points: Vec<PointKind> = kinds
            .iter()
            .map(|&(exec, a, lo, hi)| {
                if exec {
                    PointKind::Executable
                } else {
                    PointKind::Contingent { activation: TimePoint(a), lo: Ticks(lo), hi: Ticks(hi) }
                }
            })
            .collect();
        let edges: Vec<DispatchEdge> = edges
            .iter()
            .map(|&(f, t, w)| DispatchEdge { from: TimePoint(f), to: TimePoint(t), weight: Ticks(w) })
            .collect();
        let waits: Vec<Wait> = waits
            .iter()
            .map(|&(x, c, d)| Wait { waiter: TimePoint(x), contingent: TimePoint(c), delay: Ticks(d) })
            .collect();

        let Ok(mut d) = Dispatcher::<6, 24>::new(&points, &edges, &waits) else {
            return Ok(());
        };

        let mut out = [TimePoint(0); 6];

        for call in calls {
            let _ = match call {
                Call::Observe(p, t) => d.observe(TimePoint(p), Ticks(t)),
                Call::Execute(p, t) => d.execute(TimePoint(p), Ticks(t)),
                Call::Step(p, t) => d.step(Ticks(t), &[TimePoint(p)], &mut out).map(|_| ()),
                Call::Deadlines(t) => d.check_deadlines(Ticks(t)),
            };
            let _ = (d.next_wakeup(), d.is_complete(), d.window(TimePoint(0)), d.time(TimePoint(0)));
        }
    }
}
