//! Dispatchable STNs: equivalence of the filtered network and simulated dispatch.

use proptest::prelude::*;
use temporal_control::{
    Budget, DispatchEdge, DispatchError, DispatchableNetwork, Dispatcher, PointKind, Stn, Ticks,
    TimePoint, Verdict,
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
