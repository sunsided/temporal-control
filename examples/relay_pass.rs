//! A lander downlinks a sample analysis through a relay pass whose time is uncertain.
//!
//! Neither the analysis duration nor the pass opening is known in advance: both are
//! contingent links. The ground checks that the plan is dynamically controllable (it works
//! however both turn out), sees an explanation when a mission rule makes it fail, compiles the
//! plan into a dispatchable network, and uplinks it as a checksummed byte string. Onboard, a
//! fixed-capacity dispatcher without heap allocation loads the bytes in place and executes
//! the plan tick by tick, reacting to the analysis end and the pass opening as they happen.
//!
//! Run with `cargo run --example relay_pass`. Times are minutes after the epoch.

use temporal_control::{
    Budget, Controllability, DispatchableNetwork, Dispatcher, PointKind, Stnu, Ticks, TimePoint,
};

/// Point capacity of the onboard dispatcher.
const POINTS: usize = 6;
/// Edge-entry capacity of the onboard dispatcher.
const ENTRIES: usize = 64;
/// The onboard dispatcher type: fixed capacity, `u16` handles, no heap.
type Onboard = Dispatcher<POINTS, ENTRIES, u16>;

/// The plan's time points, by role.
struct Points {
    /// The epoch every other time refers to.
    epoch: TimePoint,
    /// The analysis starts.
    analysis_start: TimePoint,
    /// The analysis ends (contingent: 20..35 minutes after its start).
    analysis_end: TimePoint,
    /// The relay pass opens (contingent: 60..90 minutes after the epoch).
    pass_open: TimePoint,
    /// The lander sends the results.
    send: TimePoint,
    /// The lander switches the radio heater off.
    heater_off: TimePoint,
}

fn main() {
    let budget = Budget::UNLIMITED;
    let (stnu, p) = build_plan();

    let net = match stnu.dispatchable(&budget).expect("small plan") {
        Controllability::Controllable(net) => net,
        Controllability::Uncontrollable(cycle) => panic!("not controllable: {cycle:?}"),
    };
    println!(
        "Controllable: {} edges and {} waits to uplink.",
        net.edges().len(),
        net.waits().len()
    );

    explain_tight_deadline(&stnu, &p);

    let bytes = net.encode().expect("small network");
    println!("Uplink: {} bytes, CRC-32 checked.", bytes.len());
    assert_eq!(DispatchableNetwork::decode(&bytes).as_ref(), Ok(&net));

    for (analysis, open) in [(20, 60), (35, 60), (20, 90), (35, 90), (28, 75)] {
        let mut onboard = Onboard::EMPTY;
        onboard.load_encoded(&bytes).expect("valid uplink");
        reject_corrupted_uplink(&mut onboard, &bytes);

        let times = execute(&mut onboard, &net, &p, analysis, open);
        check(&times, &p, open);
        println!(
            "  analysis {analysis:>2} min, pass opens at {open:>2}: sent at {:>3}, heater off at {:>3}",
            times[p.send.0].0, times[p.heater_off.0].0
        );
    }
}

/// Builds the lander plan as an STNU.
fn build_plan() -> (Stnu, Points) {
    let mut stnu = Stnu::new();
    let epoch = stnu.add_time_point().expect("fits");
    let analysis_start = stnu.add_time_point().expect("fits");
    let analysis_end = stnu
        .add_contingent_link(analysis_start, Ticks(20), Ticks(35))
        .expect("valid link");
    // The communication window: its opening is a contingent point of the epoch.
    let pass_open = stnu
        .add_contingent_link(epoch, Ticks(60), Ticks(90))
        .expect("valid link");
    let send = stnu.add_time_point().expect("fits");
    let heater_off = stnu.add_time_point().expect("fits");
    let p = Points {
        epoch,
        analysis_start,
        analysis_end,
        pass_open,
        send,
        heater_off,
    };

    let requirements = [
        (p.epoch, p.analysis_start, Some(0), Some(10)),
        // Package the results first.
        (p.analysis_end, p.send, Some(2), None),
        // Send while the pass lasts: 12 minutes after it opens.
        (p.pass_open, p.send, Some(0), Some(12)),
        (p.send, p.heater_off, Some(1), Some(5)),
    ];

    for (from, to, lo, hi) in requirements {
        stnu.add_constraint(from, to, lo.map(Ticks), hi.map(Ticks))
            .expect("exists");
    }

    (stnu, p)
}

/// A mission rule "results down by minute 80" fails when the pass opens late, and the
/// explanation names the pass's contingent link.
fn explain_tight_deadline(stnu: &Stnu, p: &Points) {
    let mut tight = stnu.clone();
    tight
        .add_constraint(p.epoch, p.send, None, Some(Ticks(80)))
        .expect("exists");

    let verdict = tight
        .check_controllability(&Budget::UNLIMITED)
        .expect("small plan");
    let Controllability::Uncontrollable(cycle) = verdict else {
        panic!("a pass opening at 90 misses the deadline at 80");
    };

    let links = cycle.contingent_links();
    let pass_link = tight
        .links()
        .iter()
        .position(|l| l.contingent == p.pass_open)
        .expect("pass link");
    println!(
        "With 'send by minute 80': not controllable; cycle of weight {} through contingent links {links:?}.",
        cycle.weight().0
    );
    assert!(links.iter().any(|l| l.0 == pass_link));
}

/// Loading corrupted bytes fails and leaves the loaded plan as it was.
fn reject_corrupted_uplink(onboard: &mut Onboard, bytes: &[u8]) {
    let before = onboard.window(TimePoint(0));
    let mut corrupted = bytes.to_vec();
    corrupted[bytes.len() / 2] ^= 0x10;

    assert!(onboard.load_encoded(&corrupted).is_err());
    assert_eq!(onboard.window(TimePoint(0)), before);
}

/// Runs the dispatcher tick by tick with the given contingent durations.
///
/// Returns the time of every point.
fn execute(
    onboard: &mut Onboard,
    net: &DispatchableNetwork,
    p: &Points,
    analysis: i64,
    open: i64,
) -> Vec<Ticks> {
    let duration = |c: TimePoint| if c == p.analysis_end { analysis } else { open };
    let mut due: Vec<Option<i64>> = vec![None; net.points().len()];
    let mut out = [TimePoint(0); POINTS];

    for t in 0..=200 {
        let observed: Vec<TimePoint> = (0..due.len())
            .filter(|&c| due[c] == Some(t))
            .map(TimePoint)
            .collect();
        let executed = onboard
            .step(Ticks(t), &observed, &mut out)
            .expect("a dispatchable network never fails");

        for &started in out[..executed].iter().chain(&observed) {
            start_contingents(net, started, t, duration, &mut due);
        }

        if onboard.is_complete() {
            return (0..net.points().len())
                .map(|i| onboard.time(TimePoint(i)).expect("complete"))
                .collect();
        }
    }

    panic!("the plan did not complete");
}

/// Schedules nature's contingent points that `started` activates at `t`.
fn start_contingents(
    net: &DispatchableNetwork,
    started: TimePoint,
    t: i64,
    duration: impl Fn(TimePoint) -> i64,
    due: &mut [Option<i64>],
) {
    for (c, kind) in net.points().iter().enumerate() {
        if matches!(*kind, PointKind::Contingent { activation, .. } if activation == started) {
            due[c] = Some(t + duration(TimePoint(c)));
        }
    }
}

/// Checks the requirements on the executed times.
fn check(times: &[Ticks], p: &Points, open: i64) {
    let at = |tp: TimePoint| times[tp.0].0;

    assert!(
        open <= at(p.send) && at(p.send) <= open + 12,
        "sent outside the pass"
    );
    assert!(
        at(p.analysis_end) + 2 <= at(p.send),
        "sent before packaging"
    );
    assert!((1..=5).contains(&(at(p.heater_off) - at(p.send))));
    assert!((0..=10).contains(&(at(p.analysis_start) - at(p.epoch))));
}

#[cfg(test)]
mod tests {
    /// The example runs to completion; every assertion in it holds.
    #[test]
    fn runs() {
        super::main();
    }
}
