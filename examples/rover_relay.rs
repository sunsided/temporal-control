//! Two rovers and an orbiter share one plan but rarely talk to each other.
//!
//! Rover A drives and drills, rover B images the drill site while the dust settles, and both
//! hand their data to an orbiter during its relay pass. The ground decouples the plan once and
//! uplinks each vehicle its part; each vehicle then executes, and absorbs small changes,
//! without asking anybody. A change beyond its decoupled window waits for the next contact
//! between the vehicles (here a short UHF contact before the sol starts), where they exchange
//! summaries, check the change together and redistribute the slack, still without a central
//! solver.
//!
//! Run with `cargo run --example rover_relay`. Times are minutes after the start of the sol.

use temporal_control::{
    Agent, AgentGroup, AgentId, AgentParts, Budget, GroupVerdict, MultiAgentStn, Stn, Summary,
    Ticks, TimePoint, Verdict,
};

/// Rover A: drives, drills, sends its sample data.
const ROVER_A: AgentId = AgentId(0);
/// Rover B: images the drill site, sends the images.
const ROVER_B: AgentId = AgentId(1);
/// The orbiter: points its relay antenna at the site for one pass.
const ORBITER: AgentId = AgentId(2);

/// The plan's time points, by role.
struct Points {
    /// Rover A arrives at the drill site.
    drive_end: TimePoint,
    /// Rover A starts drilling.
    drill_start: TimePoint,
    /// Rover A stops drilling; the dust starts to settle.
    drill_end: TimePoint,
    /// Rover A sends its data to the orbiter.
    a_send: TimePoint,
    /// Rover B images the drill site.
    image: TimePoint,
    /// Rover B sends its images to the orbiter.
    b_send: TimePoint,
    /// The orbiter's relay pass starts.
    pass: TimePoint,
}

fn main() {
    let budget = Budget::UNLIMITED;
    let (plan, p) = build_plan();

    let Verdict::Consistent(decoupled) = plan.decouple(&budget).expect("small plan") else {
        panic!("the plan is consistent");
    };
    let mut agents = uplink(decoupled);

    println!("Decoupled windows (minutes after sol start):");
    print_windows(&agents, &p);
    report_flexibility(&plan, &agents);

    println!("\nIf each vehicle executed alone: rover A as early as it can, the others late.");
    execute_independently(&plan, &agents);

    println!("\nBefore the sol starts, rover A's arm check delays drilling.");
    absorb_locally(&mut agents[ROVER_A.0], &p);

    println!(
        "\nContact before the sol starts (nothing has executed): the vehicles exchange summaries."
    );
    redecouple_with_proposal(&mut agents, &p);
    print_windows(&agents, &p);

    let mut grown = plan.clone();
    let z = grown.reference();
    grown
        .add_constraint(p.drill_start, p.drill_end, Some(Ticks(30)), None)
        .expect("exists");
    grown
        .add_constraint(z, p.drill_start, Some(Ticks(80)), None)
        .expect("exists");
    execute_independently(&grown, &agents);

    println!("\nRover B proposes imaging before the drill can be done.");
    reject_conflicting_proposal(&mut agents, &p);
}

/// Sends every vehicle its part as plain data and rebuilds it there.
///
/// The fields of [`AgentParts`] are what a wire format would carry; the vehicle validates
/// them and checks its local network on arrival.
fn uplink(agents: Vec<Agent>) -> Vec<Agent> {
    agents
        .into_iter()
        .map(|agent| {
            let parts: AgentParts = agent.into_parts();
            match Agent::from_parts(parts, &Budget::UNLIMITED).expect("well formed") {
                Verdict::Consistent(agent) => agent,
                Verdict::Inconsistent(_) => panic!("a decoupled part is consistent"),
            }
        })
        .collect()
}

/// Builds the joint plan of all three vehicles.
fn build_plan() -> (MultiAgentStn, Points) {
    let mut plan = MultiAgentStn::new(3);
    let z = plan.reference();
    let mut point = |agent| plan.add_time_point(agent).expect("known agent");
    let p = Points {
        drive_end: point(ROVER_A),
        drill_start: point(ROVER_A),
        drill_end: point(ROVER_A),
        a_send: point(ROVER_A),
        image: point(ROVER_B),
        b_send: point(ROVER_B),
        pass: point(ORBITER),
    };

    let constraints = [
        // Rover A: drive, drill, compress the data.
        (z, p.drive_end, Some(30), Some(60)),
        (p.drive_end, p.drill_start, Some(5), Some(30)),
        (p.drill_start, p.drill_end, Some(20), Some(40)),
        (p.drill_end, p.a_send, Some(5), None),
        // Rover B: image during daylight, then compress.
        (z, p.image, Some(0), Some(240)),
        (p.image, p.b_send, Some(5), None),
        // The orbiter's pass geometry.
        (z, p.pass, Some(150), Some(200)),
        // Shared: image once the dust settled, before the shadows move.
        (p.drill_end, p.image, Some(15), Some(45)),
        // Shared: rover A's relay slot, then rover B's.
        (p.pass, p.a_send, Some(0), Some(10)),
        (p.pass, p.b_send, Some(10), Some(20)),
    ];

    for (from, to, lo, hi) in constraints {
        plan.add_constraint(from, to, lo.map(Ticks), hi.map(Ticks))
            .expect("exists");
    }

    (plan, p)
}

/// The window `(earliest, latest)` of `tp` in `agent`'s local network.
fn window(agent: &Agent, tp: TimePoint) -> (Option<Ticks>, Option<Ticks>) {
    let local = agent.local_network().expect("fits");
    let Verdict::Consistent(min) = local.minimal_network(&Budget::UNLIMITED).expect("fits") else {
        panic!("a local network is always consistent");
    };
    let z = TimePoint(0);
    let tp = agent.local_point(tp).expect("own point");
    (min.lower(z, tp), min.upper(z, tp))
}

/// Prints every vehicle's window for each of its points.
fn print_windows(agents: &[Agent], p: &Points) {
    let named = [
        (ROVER_A, "drive end", p.drive_end),
        (ROVER_A, "drill start", p.drill_start),
        (ROVER_A, "drill end", p.drill_end),
        (ROVER_A, "A send", p.a_send),
        (ROVER_B, "image", p.image),
        (ROVER_B, "B send", p.b_send),
        (ORBITER, "pass start", p.pass),
    ];

    for (agent, name, tp) in named {
        let (lo, hi) = window(&agents[agent.0], tp);
        println!("  {name:<12} [{:>4}, {:>4}]", show(lo), show(hi));
    }
}

/// Formats an optional bound.
fn show(t: Option<Ticks>) -> String {
    t.map_or_else(|| "inf".into(), |t| t.0.to_string())
}

/// Compares the flexibility the agents keep with that of the undivided plan.
///
/// Flexibility is the sum of the window widths of all points. The undivided plan's windows
/// cannot all be used at once (choosing one point narrows the others); the decoupled ones can.
fn report_flexibility(plan: &MultiAgentStn, agents: &[Agent]) {
    let Verdict::Consistent(min) = plan
        .stn()
        .minimal_network(&Budget::UNLIMITED)
        .expect("fits")
    else {
        panic!("consistent");
    };
    let z = plan.reference();
    let width = |(lo, hi): (Option<Ticks>, Option<Ticks>)| hi.zip(lo).map_or(0, |(h, l)| h.0 - l.0);

    let joint: i64 = (1..plan.stn().time_point_count())
        .map(|i| width((min.lower(z, TimePoint(i)), min.upper(z, TimePoint(i)))))
        .sum();
    let kept: i64 = agents
        .iter()
        .flat_map(|agent| {
            agent
                .points()
                .iter()
                .map(move |&tp| width(window(agent, tp)))
        })
        .sum();

    println!("Flexibility: {kept} of {joint} window-minutes, usable independently.");
    assert!(0 < kept && kept <= joint);
}

/// Lets every vehicle fix its own times alone and checks the combination against `plan`.
fn execute_independently(plan: &MultiAgentStn, agents: &[Agent]) {
    let mut times = vec![Ticks(0); plan.stn().time_point_count()];

    for agent in agents {
        let earliest = agent.id() == ROVER_A;

        for &tp in agent.points() {
            // The earliest (or latest) end of every window together is a schedule of an STN.
            let (lo, hi) = window(agent, tp);
            times[tp.0] = if earliest { lo } else { hi }.expect("bounded window");
        }
    }

    assert!(satisfies(plan.stn(), &times), "decoupling is unsound");
    let shown: Vec<i64> = times[1..].iter().map(|t| t.0).collect();
    println!("  times {shown:?}: every constraint of the plan holds");
}

/// Whether `times` satisfies every constraint of `stn`.
fn satisfies(stn: &Stn, times: &[Ticks]) -> bool {
    stn.constraints().iter().all(|c| {
        let diff = times[c.to.0].0 - times[c.from.0].0;
        c.lo.is_none_or(|lo| lo.0 <= diff) && c.hi.is_none_or(|hi| diff <= hi.0)
    })
}

/// Rover A absorbs a slower drill inside its window, but not a late drill start.
fn absorb_locally(rover: &mut Agent, p: &Points) {
    let budget = Budget::UNLIMITED;
    let z = TimePoint(0);
    let slower = rover
        .add_constraint(p.drill_start, p.drill_end, Some(Ticks(30)), None, &budget)
        .expect("own points");
    println!("  drilling takes 30+ minutes:      {}", verdict(&slower));
    assert_eq!(slower, Verdict::Consistent(()));

    let late = rover
        .add_constraint(z, p.drill_start, Some(Ticks(80)), None, &budget)
        .expect("own points");
    println!("  drilling starts after minute 80: {}", verdict(&late));
    assert!(matches!(late, Verdict::Inconsistent(_)));

    rover
        .propose(z, p.drill_start, Some(Ticks(80)), None)
        .expect("own point");
    println!("  proposed for the next contact");
}

/// Describes a local verdict.
fn verdict(v: &Verdict<()>) -> &'static str {
    match v {
        Verdict::Consistent(()) => "accepted locally, nobody asked",
        Verdict::Inconsistent(_) => "beyond the decoupled window",
    }
}

/// Every vehicle summarises itself for the whole group.
fn summaries(agents: &[Agent]) -> Vec<Summary> {
    let group = [ROVER_A, ROVER_B, ORBITER];
    agents
        .iter()
        .map(
            |agent| match agent.summary(&group, &Budget::UNLIMITED).expect("fits") {
                Verdict::Consistent(summary) => summary,
                Verdict::Inconsistent(_) => panic!("proposals fit each vehicle alone"),
            },
        )
        .collect()
}

/// The group accepts rover A's proposal and every vehicle adopts the new cuts.
fn redecouple_with_proposal(agents: &mut [Agent], p: &Points) {
    let budget = Budget::UNLIMITED;
    let summaries = summaries(agents);
    let shared: usize = summaries.iter().map(|s| s.bounds().len()).sum();
    println!("  {} summaries, {shared} bounds in total", summaries.len());

    // Every vehicle builds the same group from the same messages.
    let group = AgentGroup::new(&summaries).expect("complete set");
    assert_eq!(
        group.check_consistency(&budget),
        Ok(GroupVerdict::Consistent)
    );

    for agent in agents.iter_mut() {
        assert_eq!(
            agent.redecouple(&group, &budget),
            Ok(GroupVerdict::Consistent)
        );
    }

    let rover = &agents[ROVER_A.0];
    assert_eq!(rover.pending(), &[]);
    let (lo, _) = window(rover, p.drill_end);
    println!("  adopted; drill end now from minute {}", show(lo));
}

/// The group rejects a proposal that conflicts with the other rover's plan.
fn reject_conflicting_proposal(agents: &mut [Agent], p: &Points) {
    let budget = Budget::UNLIMITED;
    let z = TimePoint(0);
    agents[ROVER_B.0]
        .propose(z, p.image, None, Some(Ticks(70)))
        .expect("own point");

    let group = AgentGroup::new(&summaries(agents)).expect("complete set");
    let GroupVerdict::Inconsistent(conflict) = group.check_consistency(&budget).expect("fits")
    else {
        panic!("the drill cannot end before minute 55");
    };
    println!(
        "  conflict: shared constraints {:?}, plans of agents {:?}",
        conflict.shared, conflict.agents
    );
    assert!(conflict.agents.contains(&ROVER_A) && conflict.agents.contains(&ROVER_B));

    let cuts = agents[ROVER_B.0].cuts().to_vec();
    for agent in agents.iter_mut() {
        assert!(matches!(
            agent.redecouple(&group, &budget),
            Ok(GroupVerdict::Inconsistent(_))
        ));
    }

    assert_eq!(agents[ROVER_B.0].pending(), &[]);
    assert_eq!(agents[ROVER_B.0].cuts(), cuts.as_slice());
    println!("  proposal dropped; every vehicle keeps its cuts");
}

#[cfg(test)]
mod tests {
    /// The example runs to completion; every assertion in it holds.
    #[test]
    fn runs() {
        super::main();
    }
}
