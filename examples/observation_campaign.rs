//! A telescope night planned interactively, then a long survey checked without the full
//! all-pairs matrix.
//!
//! Operators add the constraints of a night one by one; the incremental network keeps every
//! window current after each one and rejects a request that cannot fit, naming the
//! constraints it conflicts with. A 200-field survey is then checked with P³C, which only
//! computes the bounds of constrained pairs, and the night is compiled into a dispatchable
//! network that a fixed-capacity dispatcher executes.
//!
//! Run with `cargo run --example observation_campaign`. Times are minutes after dusk.

use temporal_control::{
    Budget, ConstraintId, Dispatcher, IncrementalStn, NegativeCycle, Stn, Ticks, TimePoint, Verdict,
};

/// Number of survey fields.
const FIELDS: usize = 200;

fn main() {
    let night = plan_night();
    survey();
    dispatch_night(&night);
}

/// Dusk, the reference of the night.
const DUSK: usize = 0;
/// The opening calibration.
const CALIB: usize = 1;
/// Target 1 exposure starts.
const T1_START: usize = 2;
/// Target 1 exposure ends.
const T1_END: usize = 3;
/// Target 2 exposure starts.
const T2_START: usize = 4;
/// Target 2 exposure ends.
const T2_END: usize = 5;
/// Dawn.
const DAWN: usize = 6;

/// One operator request: `lo <= t_to - t_from <= hi`, in minutes.
type Request = (&'static str, usize, usize, Option<i64>, Option<i64>);

/// The operators' requests, in the order they arrive.
const REQUESTS: [Request; 9] = [
    ("night lasts 600 minutes", DUSK, DAWN, Some(600), Some(600)),
    (
        "calibrate within 30 minutes of dusk",
        DUSK,
        CALIB,
        Some(0),
        Some(30),
    ),
    (
        "target 1 after calibration",
        CALIB,
        T1_START,
        Some(10),
        None,
    ),
    (
        "target 1 exposes 120..180",
        T1_START,
        T1_END,
        Some(120),
        Some(180),
    ),
    ("slew to target 2", T1_END, T2_START, Some(15), None),
    (
        "target 2 exposes 200..240",
        T2_START,
        T2_END,
        Some(200),
        Some(240),
    ),
    (
        "closing calibration before dawn",
        T2_END,
        DAWN,
        Some(30),
        None,
    ),
    (
        "PI: target 1 after minute 240",
        DUSK,
        T1_START,
        Some(240),
        None,
    ),
    (
        "PI: target 1 after minute 200",
        DUSK,
        T1_START,
        Some(200),
        None,
    ),
];

/// Adds the night's constraints one at a time and returns the accepted plan.
fn plan_night() -> Stn {
    let budget = Budget::UNLIMITED;
    let mut inc = IncrementalStn::new();

    for _ in 0..=DAWN {
        inc.add_time_point(&budget).expect("fits");
    }

    let mut names: Vec<&str> = Vec::new();

    for (name, from, to, lo, hi) in REQUESTS {
        let verdict = inc
            .add_constraint(
                TimePoint(from),
                TimePoint(to),
                lo.map(Ticks),
                hi.map(Ticks),
                &budget,
            )
            .expect("within budget");

        match verdict {
            Verdict::Consistent(id) => {
                assert_eq!(id.0, names.len());
                names.push(name);
                let (dusk, end) = (TimePoint(DUSK), TimePoint(T2_END));
                println!(
                    "accepted {name:<38} target 2 ends in {}",
                    window(inc.lower(dusk, end), inc.upper(dusk, end))
                );
            }
            Verdict::Inconsistent(cycle) => explain_rejection(name, &cycle, &names),
        }
    }

    assert_eq!(
        inc.lower(TimePoint(DUSK), TimePoint(T1_START)),
        Some(Ticks(200))
    );
    inc.stn().clone()
}

/// Prints the accepted requests a rejected one conflicts with.
fn explain_rejection(name: &str, cycle: &NegativeCycle, names: &[&str]) {
    println!("rejected {name}: conflicts with");
    let mut ids: Vec<ConstraintId> = cycle.edges().iter().map(|e| e.constraint).collect();
    ids.sort_unstable();
    ids.dedup();

    for id in &ids {
        // The rejected request gets the next id.
        println!("  - {}", names.get(id.0).copied().unwrap_or(name));
    }

    assert!(
        ids.contains(&ConstraintId(names.len())),
        "names the request"
    );
    assert_eq!(name, "PI: target 1 after minute 240");
}

/// Formats a window; missing bounds are infinite.
fn window(lo: Option<Ticks>, hi: Option<Ticks>) -> String {
    let lo = lo.map_or_else(|| "-inf".into(), |t| t.0.to_string());
    let hi = hi.map_or_else(|| "inf".into(), |t| t.0.to_string());
    format!("[{lo}, {hi}]")
}

/// Checks a long, sparse survey with P³C and compares it with the full minimal network.
fn survey() {
    let budget = Budget::UNLIMITED;
    let mut stn = Stn::new();
    let points: Vec<TimePoint> = (0..=FIELDS)
        .map(|_| stn.add_time_point().expect("fits"))
        .collect();

    for pair in points.windows(2) {
        // Expose and slew to the next field.
        stn.add_constraint(pair[0], pair[1], Some(Ticks(8)), Some(Ticks(12)))
            .expect("exists");
    }

    for i in (0..FIELDS - 25).step_by(25) {
        // Revisit the standard star: at most 280 minutes per block of 25 fields.
        stn.add_constraint(points[i], points[i + 25], None, Some(Ticks(280)))
            .expect("exists");
    }

    let Verdict::Consistent(sparse) = stn.partial_path_consistency(&budget).expect("fits") else {
        panic!("the survey is consistent");
    };
    let Verdict::Consistent(full) = stn.minimal_network(&budget).expect("fits") else {
        panic!("the survey is consistent");
    };

    let pairs: Vec<(TimePoint, TimePoint)> = sparse.pairs().collect();

    for &(a, b) in &pairs {
        assert_eq!(
            (sparse.lower(a, b), sparse.upper(a, b)),
            (full.lower(a, b), full.upper(a, b))
        );
    }

    let cells = points.len() * points.len();
    println!(
        "\nSurvey of {FIELDS} fields: P³C computed {} pairs, Floyd-Warshall {cells} cells; bounds agree.",
        pairs.len()
    );
    assert!(pairs.len() * 10 < cells);

    let last = points[FIELDS];
    println!(
        "  the survey takes {} minutes",
        window(full.lower(points[0], last), full.upper(points[0], last))
    );
}

/// Compiles the night into a dispatchable network and runs it as soon as possible.
fn dispatch_night(night: &Stn) {
    let n = night.time_point_count();
    let Verdict::Consistent(net) = night.dispatchable(&Budget::UNLIMITED).expect("fits") else {
        panic!("the night is consistent");
    };
    println!(
        "\nNight: {} dispatchable edges instead of {} in the minimal network.",
        net.edges().len(),
        n * (n - 1)
    );
    assert!(net.edges().len() < n * (n - 1));

    let mut dispatcher: Dispatcher<7, 64> = net.dispatcher().expect("fits");
    let mut out = [TimePoint(0); 7];
    let mut t = 0;

    loop {
        dispatcher
            .step(Ticks(t), &[], &mut out)
            .expect("dispatchable");

        if dispatcher.is_complete() {
            break;
        }

        let wakeup = dispatcher
            .next_wakeup()
            .expect("an unexecuted point can still run");
        t = wakeup.0.max(t + 1);
    }

    let times: Vec<i64> = (0..n)
        .map(|i| dispatcher.time(TimePoint(i)).expect("complete").0)
        .collect();
    println!("  executed at {times:?}");
    assert!(night.constraints().iter().all(|c| {
        let diff = times[c.to.0] - times[c.from.0];
        c.lo.is_none_or(|lo| lo.0 <= diff) && c.hi.is_none_or(|hi| diff <= hi.0)
    }));
}

#[cfg(test)]
mod tests {
    /// The example runs to completion; every assertion in it holds.
    #[test]
    fn runs() {
        super::main();
    }
}
