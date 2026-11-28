//! Multi-agent STNs: decoupling soundness, distributed checks and re-decoupling against the
//! central plan.

use proptest::prelude::*;
use temporal_control::{
    Agent, AgentGroup, AgentId, Budget, Constraint, GroupVerdict, IncrementalStn, MultiAgentStn,
    Stn, Summary, Ticks, TimePoint, Verdict,
};

use crate::generators::ConstraintSpec;
use crate::sim::Rng;

/// A multi-agent plan as plain data. Point `0` is the reference; point `i > 0` belongs to
/// agent `owners[i - 1]`.
#[derive(Debug, Clone)]
struct PlanSpec {
    /// Number of agents. At least one.
    agents: usize,
    /// Owner of each non-reference point, below `agents`.
    owners: Vec<usize>,
    /// Constraints on points below `owners.len() + 1`.
    constraints: Vec<ConstraintSpec>,
    /// Further constraints the agents propose later; only those local to one agent are used.
    extras: Vec<ConstraintSpec>,
}

impl PlanSpec {
    /// Builds the plan with the base constraints and the given extra constraints.
    fn build(&self, extras: &[ConstraintSpec]) -> MultiAgentStn {
        let mut plan = MultiAgentStn::new(self.agents);

        for &owner in &self.owners {
            plan.add_time_point(AgentId(owner)).unwrap();
        }

        for &(from, to, lo, hi) in self.constraints.iter().chain(extras) {
            plan.add_constraint(TimePoint(from), TimePoint(to), lo.map(Ticks), hi.map(Ticks))
                .unwrap();
        }

        plan
    }

    /// The agent an extra constraint is local to, or `None` if it spans two agents or touches
    /// only the reference.
    fn local_agent(&self, (from, to, _, _): ConstraintSpec) -> Option<usize> {
        let owner = |p: usize| p.checked_sub(1).map(|i| self.owners[i]);

        match (owner(from), owner(to)) {
            (Some(a), Some(b)) if a != b => None,
            (Some(a), _) | (_, Some(a)) => Some(a),
            // A plan gives reference-only constraints to every agent; no single agent proposes them.
            (None, None) => None,
        }
    }

    /// The extra constraints local to an agent in `members`, with that agent.
    fn proposals(&self, members: &[usize]) -> Vec<(usize, ConstraintSpec)> {
        self.extras
            .iter()
            .filter_map(|&c| self.local_agent(c).map(|a| (a, c)))
            .filter(|(a, _)| members.contains(a))
            .collect()
    }
}

/// Plans with `agents` agents and up to 6 owned points. If `consistent`, every bound contains the
/// difference of a hidden schedule; otherwise bounds are random and often conflict.
fn plan(
    consistent: bool,
    agents: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = PlanSpec> {
    (agents, 0usize..=6).prop_flat_map(move |(agents, points)| {
        let n = points + 1;
        let owners = proptest::collection::vec(0..agents, points);
        let schedule = proptest::collection::vec(-40i64..=40, n);
        let raw = (
            0..n,
            0..n,
            proptest::option::weighted(0.8, -20i64..=20),
            proptest::option::weighted(0.8, -20i64..=20),
        );
        let constraints = proptest::collection::vec(raw.clone(), 0..12);
        let extras = proptest::collection::vec(raw, 0..4);

        (owners, schedule, constraints, extras).prop_map(
            move |(owners, schedule, constraints, extras)| {
                let fit = |(from, to, lo, hi): ConstraintSpec| -> ConstraintSpec {
                    if !consistent {
                        return (from, to, lo, hi);
                    }

                    let time = |p: usize| if p == 0 { 0 } else { schedule[p] };
                    let diff = time(to) - time(from);
                    (
                        from,
                        to,
                        lo.map(|l| diff - l.abs()),
                        hi.map(|h| diff + h.abs()),
                    )
                };

                PlanSpec {
                    agents,
                    owners,
                    constraints: constraints.into_iter().map(fit).collect(),
                    extras: extras.into_iter().map(fit).collect(),
                }
            },
        )
    })
}

/// How an agent picks its own schedule from its local network.
#[derive(Debug, Clone, Copy)]
enum Policy {
    /// Every point at the start of its window.
    Earliest,
    /// Every point at the end of its window.
    Latest,
    /// Every point anywhere in its window, from this seed.
    Random(u64),
}

/// Every agent's policy, by agent.
fn policies() -> impl Strategy<Value = Vec<Policy>> {
    let policy = prop_oneof![
        Just(Policy::Earliest),
        Just(Policy::Latest),
        any::<u64>().prop_map(Policy::Random),
    ];
    proptest::collection::vec(policy, 3)
}

/// Picks a value in `[lo, hi]`; infinite ends are taken 30 ticks beyond the other end.
fn pick(lo: Option<Ticks>, hi: Option<Ticks>, policy: Policy, rng: &mut Rng) -> Ticks {
    let lo = lo.map_or_else(|| hi.map_or(-30, |h| h.0 - 30), |l| l.0);
    let hi = hi.map_or(lo + 30, |h| h.0);

    Ticks(match policy {
        Policy::Earliest => lo,
        Policy::Latest => hi,
        Policy::Random(_) => rng.range(lo, hi),
    })
}

/// One schedule of `agent`'s local network chosen by `policy`, as `(plan point, time)` pairs,
/// with the reference at zero.
fn local_schedule(agent: &Agent, policy: Policy) -> Vec<(TimePoint, Ticks)> {
    let budget = Budget::UNLIMITED;
    let local = agent.local_network().unwrap();
    let mut inc = IncrementalStn::new();
    // xorshift never leaves zero, so the low bit is forced on.
    let mut rng = Rng(match policy {
        Policy::Random(seed) => seed | 1,
        _ => 1,
    });

    for _ in 0..local.time_point_count() {
        inc.add_time_point(&budget).unwrap();
    }

    for c in local.constraints() {
        let verdict = inc
            .add_constraint(c.from, c.to, c.lo, c.hi, &budget)
            .unwrap();
        assert!(
            matches!(verdict, Verdict::Consistent(_)),
            "local network inconsistent"
        );
    }

    let z = TimePoint(0);
    let mut out = Vec::new();

    for p in (1..local.time_point_count()).map(TimePoint) {
        let t = pick(inc.lower(z, p), inc.upper(z, p), policy, &mut rng);
        let verdict = inc.add_constraint(z, p, Some(t), Some(t), &budget).unwrap();
        assert!(
            matches!(verdict, Verdict::Consistent(_)),
            "window value rejected"
        );
        out.push((agent.plan_point(p).unwrap(), t));
    }

    out
}

/// Merges every agent's own schedule and checks every constraint of `stn` on the result.
fn assert_combined_schedule_holds(stn: &Stn, agents: &[Agent], policies: &[Policy]) {
    let mut times = vec![None; stn.time_point_count()];
    times[0] = Some(Ticks(0));

    for (agent, &policy) in agents.iter().zip(policies.iter().cycle()) {
        for (p, t) in local_schedule(agent, policy) {
            times[p.0] = Some(t);
        }
    }

    for c in stn.constraints() {
        assert_holds(c, &times);
    }
}

/// Asserts `lo <= t_to - t_from <= hi` on the merged times.
fn assert_holds(c: &Constraint, times: &[Option<Ticks>]) {
    let diff = times[c.to.0].unwrap().0 - times[c.from.0].unwrap().0;
    assert!(c.lo.is_none_or(|lo| lo.0 <= diff), "{c:?} violated: {diff}");
    assert!(c.hi.is_none_or(|hi| diff <= hi.0), "{c:?} violated: {diff}");
}

/// Decouples a plan known to be consistent.
fn decouple(plan: &MultiAgentStn) -> Vec<Agent> {
    match plan.decouple(&Budget::UNLIMITED).unwrap() {
        Verdict::Consistent(agents) => agents,
        Verdict::Inconsistent(cycle) => panic!("inconsistent: {cycle:?}"),
    }
}

/// Lets every agent in `members` propose its extras, then collects the members' summaries;
/// `None` if a summary is already inconsistent.
fn propose_and_summarise(
    spec: &PlanSpec,
    agents: &mut [Agent],
    members: &[usize],
) -> Option<Vec<Summary>> {
    for (a, (from, to, lo, hi)) in spec.proposals(members) {
        agents[a]
            .propose(TimePoint(from), TimePoint(to), lo.map(Ticks), hi.map(Ticks))
            .unwrap();
    }

    let ids: Vec<AgentId> = members.iter().map(|&a| AgentId(a)).collect();
    members
        .iter()
        .map(
            |&a| match agents[a].summary(&ids, &Budget::UNLIMITED).unwrap() {
                Verdict::Consistent(summary) => Some(summary),
                Verdict::Inconsistent(_) => None,
            },
        )
        .collect()
}

/// The proposed extras of `members`, as plan constraint specs.
fn proposed(spec: &PlanSpec, members: &[usize]) -> Vec<ConstraintSpec> {
    spec.proposals(members)
        .into_iter()
        .map(|(_, c)| c)
        .collect()
}

proptest! {
    /// Decoupling succeeds exactly on consistent plans, and an inconsistent plan is explained.
    #[test]
    fn decouple_verdict_matches_central(spec in plan(false, 1..=3)) {
        let plan = spec.build(&[]);
        let central = plan.stn().check_consistency(&Budget::UNLIMITED).unwrap();
        let decoupled = plan.decouple(&Budget::UNLIMITED).unwrap();

        match (central, decoupled) {
            (Verdict::Consistent(_), Verdict::Consistent(agents)) => prop_assert_eq!(agents.len(), spec.agents),
            (Verdict::Inconsistent(_), Verdict::Inconsistent(cycle)) => prop_assert!(cycle.weight() < Ticks(0)),
            (c, d) => prop_assert!(false, "central {c:?}, decoupled {d:?}"),
        }
    }

    /// Schedules each agent picks alone from its local network combine into a schedule of
    /// the plan, however the agents pick.
    #[test]
    fn local_schedules_combine(spec in plan(true, 1..=3), policies in policies()) {
        let plan = spec.build(&[]);
        let agents = decouple(&plan);
        assert_combined_schedule_holds(plan.stn(), &agents, &policies);
    }

    /// With every agent in the group, the summaries decide the consistency of the plan with
    /// the proposals, as the central check does.
    #[test]
    fn distributed_check_matches_central(spec in plan(true, 1..=3)) {
        let members: Vec<usize> = (0..spec.agents).collect();
        let mut agents = decouple(&spec.build(&[]));
        let central = spec.build(&proposed(&spec, &members)).stn().check_consistency(&Budget::UNLIMITED).unwrap();
        let distributed = propose_and_summarise(&spec, &mut agents, &members)
            .map(|summaries| AgentGroup::new(&summaries).unwrap().check_consistency(&Budget::UNLIMITED).unwrap());

        let consistent = matches!(central, Verdict::Consistent(_));
        prop_assert_eq!(distributed == Some(GroupVerdict::Consistent), consistent);
    }

    /// Re-decoupling with every agent gives each agent the view a central decoupling of the
    /// plan with the proposals gives it.
    #[test]
    fn redecouple_all_matches_central(spec in plan(true, 1..=3)) {
        let members: Vec<usize> = (0..spec.agents).collect();
        let mut agents = decouple(&spec.build(&[]));
        let Some(summaries) = propose_and_summarise(&spec, &mut agents, &members) else {
            return Ok(());
        };

        let group = AgentGroup::new(&summaries).unwrap();
        let before = agents.clone();
        let verdicts: Vec<GroupVerdict> = agents
            .iter_mut()
            .map(|agent| agent.redecouple(&group, &Budget::UNLIMITED).unwrap())
            .collect();

        match spec.build(&proposed(&spec, &members)).decouple(&Budget::UNLIMITED).unwrap() {
            Verdict::Consistent(central) => {
                for (agent, expected) in agents.iter().zip(&central) {
                    prop_assert!(verdicts.iter().all(|v| *v == GroupVerdict::Consistent));
                    prop_assert_eq!(agent.cuts(), expected.cuts());
                    prop_assert_eq!(agent.constraints(), expected.constraints());
                    prop_assert!(agent.pending().is_empty());
                }
            }
            Verdict::Inconsistent(_) => {
                for (agent, old) in agents.iter().zip(&before) {
                    prop_assert!(matches!(verdicts[agent.id().0], GroupVerdict::Inconsistent(_)));
                    prop_assert_eq!(agent.cuts(), old.cuts());
                    prop_assert!(agent.pending().is_empty());
                }
            }
        }
    }

    /// Two of three agents re-decouple alone; the third keeps its cuts, and schedules picked
    /// alone afterwards still combine into a schedule of the plan with the adopted proposals.
    #[test]
    fn partial_redecouple_stays_sound(spec in plan(true, 3..=3), policies in policies()) {
        let members = [0, 1];
        let mut agents = decouple(&spec.build(&[]));
        let Some(summaries) = propose_and_summarise(&spec, &mut agents, &members) else {
            return Ok(());
        };

        let group = AgentGroup::new(&summaries).unwrap();
        let third = agents[2].clone();
        let mut adopted = true;

        for agent in &mut agents[..2] {
            adopted &= agent.redecouple(&group, &Budget::UNLIMITED).unwrap() == GroupVerdict::Consistent;
        }

        prop_assert_eq!(&agents[2], &third);
        let extras = if adopted { proposed(&spec, &members) } else { Vec::new() };
        assert_combined_schedule_holds(spec.build(&extras).stn(), &agents, &policies);
    }
}

/// Summaries from different groups, a missing member or a stale revision are refused.
#[test]
fn mismatched_summaries_refused() {
    let mut plan = MultiAgentStn::new(2);
    let z = plan.reference();
    let a = plan.add_time_point(AgentId(0)).unwrap();
    let b = plan.add_time_point(AgentId(1)).unwrap();
    plan.add_constraint(z, a, Some(Ticks(0)), Some(Ticks(10)))
        .unwrap();
    plan.add_constraint(a, b, Some(Ticks(1)), Some(Ticks(5)))
        .unwrap();
    let mut agents = decouple(&plan);

    let summary = |agent: &Agent, group: &[AgentId]| match agent
        .summary(group, &Budget::UNLIMITED)
        .unwrap()
    {
        Verdict::Consistent(s) => s,
        Verdict::Inconsistent(_) => unreachable!("no proposals"),
    };
    let both = [AgentId(0), AgentId(1)];
    let s0 = summary(&agents[0], &both);
    let s1 = summary(&agents[1], &both);
    let alone = summary(&agents[1], &[AgentId(1)]);

    assert_eq!(
        AgentGroup::new(&[]).unwrap_err(),
        temporal_control::Error::MismatchedSummaries
    );
    assert_eq!(
        AgentGroup::new(std::slice::from_ref(&s0)).unwrap_err(),
        temporal_control::Error::MismatchedSummaries
    );
    assert_eq!(
        AgentGroup::new(&[s0.clone(), alone]).unwrap_err(),
        temporal_control::Error::MismatchedSummaries
    );
    assert_eq!(
        AgentGroup::new(&[s0.clone(), s0.clone()]).unwrap_err(),
        temporal_control::Error::MismatchedSummaries
    );

    let group = AgentGroup::new(&[s1, s0]).unwrap();
    agents[0]
        .add_constraint(z, a, None, Some(Ticks(9)), &Budget::UNLIMITED)
        .unwrap();
    assert_eq!(
        agents[0].redecouple(&group, &Budget::UNLIMITED),
        Err(temporal_control::Error::MismatchedSummaries)
    );
    assert_eq!(
        agents[1].summary(&[AgentId(0)], &Budget::UNLIMITED),
        Err(temporal_control::Error::UnknownAgent)
    );
}

/// The window of `tp` relative to the reference in `agent`'s local network.
fn window(agent: &Agent, tp: TimePoint) -> (Option<Ticks>, Option<Ticks>) {
    let network = agent.local_network().unwrap();
    let Verdict::Consistent(min) = network.minimal_network(&Budget::UNLIMITED).unwrap() else {
        panic!("a local network is consistent");
    };
    let local = agent.local_point(tp).unwrap();
    (
        min.lower(TimePoint(0), local),
        min.upper(TimePoint(0), local),
    )
}

/// Every agent's summary for the group of all agents.
fn summaries(agents: &[Agent]) -> Vec<Summary> {
    let members: Vec<AgentId> = agents.iter().map(Agent::id).collect();
    agents
        .iter()
        .map(
            |agent| match agent.summary(&members, &Budget::UNLIMITED).unwrap() {
                Verdict::Consistent(s) => s,
                Verdict::Inconsistent(_) => panic!("fits the agent alone"),
            },
        )
        .collect()
}

/// A change inside the cuts is accepted locally; one beyond them is rejected locally, fits the
/// plan as a proposal, and is adopted by re-decoupling.
#[test]
fn proposal_beyond_cuts_adopted_by_redecoupling() {
    let budget = Budget::UNLIMITED;
    let mut plan = MultiAgentStn::new(2);
    let z = plan.reference();
    let drill = plan.add_time_point(AgentId(0)).unwrap();
    let image = plan.add_time_point(AgentId(1)).unwrap();
    plan.add_constraint(z, drill, Some(Ticks(0)), Some(Ticks(100)))
        .unwrap();
    plan.add_constraint(z, image, Some(Ticks(0)), Some(Ticks(100)))
        .unwrap();
    plan.add_constraint(drill, image, Some(Ticks(10)), None)
        .unwrap();
    let mut agents = decouple(&plan);

    // The cut splits the slack: drill <= 45, image >= 55.
    assert_eq!(window(&agents[0], drill), (Some(Ticks(0)), Some(Ticks(45))));
    assert_eq!(
        window(&agents[1], image),
        (Some(Ticks(55)), Some(Ticks(100)))
    );

    let inside = agents[0]
        .add_constraint(z, drill, Some(Ticks(40)), None, &budget)
        .unwrap();
    assert_eq!(inside, Verdict::Consistent(()));
    let beyond = agents[0]
        .add_constraint(z, drill, Some(Ticks(60)), None, &budget)
        .unwrap();
    assert!(matches!(beyond, Verdict::Inconsistent(_)));

    agents[0].propose(z, drill, Some(Ticks(60)), None).unwrap();
    let group = AgentGroup::new(&summaries(&agents)).unwrap();
    assert_eq!(
        group.check_consistency(&budget),
        Ok(GroupVerdict::Consistent)
    );

    for agent in &mut agents {
        assert_eq!(
            agent.redecouple(&group, &budget),
            Ok(GroupVerdict::Consistent)
        );
    }

    assert_eq!(
        window(&agents[0], drill),
        (Some(Ticks(60)), Some(Ticks(75)))
    );
    assert_eq!(
        window(&agents[1], image),
        (Some(Ticks(85)), Some(Ticks(100)))
    );
}

/// A proposal the other agent's plan cannot absorb is named by the group and dropped.
#[test]
fn conflicting_proposal_dropped() {
    let budget = Budget::UNLIMITED;
    let mut plan = MultiAgentStn::new(2);
    let z = plan.reference();
    let drill = plan.add_time_point(AgentId(0)).unwrap();
    let image = plan.add_time_point(AgentId(1)).unwrap();
    plan.add_constraint(z, image, None, Some(Ticks(100)))
        .unwrap();
    let shared = plan
        .add_constraint(drill, image, Some(Ticks(10)), None)
        .unwrap();
    let mut agents = decouple(&plan);

    agents[0].propose(z, drill, Some(Ticks(95)), None).unwrap();
    let group = AgentGroup::new(&summaries(&agents)).unwrap();

    let GroupVerdict::Inconsistent(conflict) = group.check_consistency(&budget).unwrap() else {
        panic!("drill at 95 leaves no room for image by 100");
    };
    assert_eq!(conflict.shared, vec![shared]);
    assert_eq!(conflict.agents, vec![AgentId(0), AgentId(1)]);

    let cuts = agents[0].cuts().to_vec();
    assert!(matches!(
        agents[0].redecouple(&group, &budget),
        Ok(GroupVerdict::Inconsistent(_))
    ));
    assert_eq!(agents[0].pending(), &[]);
    assert_eq!(agents[0].cuts(), cuts.as_slice());
}

/// Rebuilds a summary from its accessors.
fn rebuild(s: &Summary) -> Result<Summary, temporal_control::Error> {
    Summary::from_parts(
        s.agent(),
        s.group().to_vec(),
        s.revision(),
        s.points().to_vec(),
        s.bounds().to_vec(),
        s.shared_constraints().to_vec(),
    )
}

proptest! {
    /// Agents and summaries survive a round trip through their plain parts.
    #[test]
    fn parts_round_trip(spec in plan(true, 1..=3)) {
        let members: Vec<usize> = (0..spec.agents).collect();
        let mut agents = decouple(&spec.build(&[]));
        let summaries = propose_and_summarise(&spec, &mut agents, &members);

        for agent in &agents {
            let rebuilt = Agent::from_parts(agent.clone().into_parts(), &Budget::UNLIMITED).unwrap();
            prop_assert_eq!(rebuilt, Verdict::Consistent(agent.clone()));
        }

        for s in summaries.iter().flatten() {
            prop_assert_eq!(&rebuild(s).unwrap(), s);
        }
    }
}

/// Malformed parts are refused, and an inconsistent local network is explained.
#[test]
fn malformed_parts_refused() {
    use temporal_control::Error;

    let mut plan = MultiAgentStn::new(2);
    let z = plan.reference();
    let a = plan.add_time_point(AgentId(0)).unwrap();
    let b = plan.add_time_point(AgentId(1)).unwrap();
    plan.add_constraint(z, a, Some(Ticks(0)), Some(Ticks(10)))
        .unwrap();
    plan.add_constraint(a, b, Some(Ticks(1)), Some(Ticks(5)))
        .unwrap();
    let agents = decouple(&plan);
    let parts = agents[0].clone().into_parts();
    let rebuild_agent = |parts| Agent::from_parts(parts, &Budget::UNLIMITED);

    let mut unsorted = parts.clone();
    unsorted.points.insert(0, b);
    let mut stranger = parts.clone();
    stranger.id = AgentId(5);
    stranger.points = vec![b];
    stranger.constraints.clear();
    stranger.cuts.clear();
    let mut bad_cut = parts.clone();
    bad_cut.shared.clear();
    let mut contradiction = parts.clone();
    contradiction.constraints.push(Constraint {
        from: z,
        to: a,
        lo: Some(Ticks(11)),
        hi: None,
    });

    assert_eq!(
        rebuild_agent(parts.clone()).unwrap(),
        Verdict::Consistent(agents[0].clone())
    );
    assert!(matches!(
        rebuild_agent(unsorted),
        Err(Error::UnknownTimePoint)
    ));
    assert!(matches!(rebuild_agent(stranger), Err(Error::UnknownAgent)));
    assert!(matches!(rebuild_agent(bad_cut), Err(Error::UnknownAgent)));
    assert!(matches!(
        rebuild_agent(contradiction),
        Ok(Verdict::Inconsistent(_))
    ));
}

/// Malformed summary parts are refused.
#[test]
fn malformed_summary_parts_refused() {
    use temporal_control::Error;

    let mut plan = MultiAgentStn::new(2);
    let z = plan.reference();
    let a = plan.add_time_point(AgentId(0)).unwrap();
    let b = plan.add_time_point(AgentId(1)).unwrap();
    plan.add_constraint(a, b, Some(Ticks(1)), Some(Ticks(5)))
        .unwrap();
    let s = summaries(&decouple(&plan)).remove(0);
    let missing_member = Summary::from_parts(
        s.agent(),
        vec![s.agent()],
        s.revision(),
        s.points().to_vec(),
        s.bounds().to_vec(),
        s.shared_constraints().to_vec(),
    );
    let reference_point = Summary::from_parts(
        s.agent(),
        s.group().to_vec(),
        s.revision(),
        vec![z],
        Vec::new(),
        Vec::new(),
    );
    assert_eq!(rebuild(&s).as_ref(), Ok(&s));
    assert_eq!(missing_member, Err(Error::MismatchedSummaries));
    assert_eq!(reference_point, Err(Error::MismatchedSummaries));
}
