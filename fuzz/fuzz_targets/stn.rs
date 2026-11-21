//! STN algorithms on arbitrary small networks: no panic, and every algorithm that answers agrees.
//!
//! Checked on the network of all constraints: a Bellman-Ford schedule satisfies every
//! constraint; Floyd-Warshall, P³C and the dispatchable filter reach the same verdict as
//! Bellman-Ford; P³C's bounds equal the minimal network's on every pair it covers; a
//! dispatchable network loads into a dispatcher and survives an encoding round trip.
//! Replaying the same steps into an [`IncrementalStn`] accepts exactly the constraints that keep
//! the accepted ones consistent, and its matrix matches a from-scratch minimal network after
//! every step. Overflow errors end a check (extreme bounds are part of the input space);
//! `Error::Internal` and other errors are failures.

#![no_main]

use libfuzzer_sys::arbitrary::{self, Arbitrary};
use libfuzzer_sys::fuzz_target;
use temporal_control::{
    Budget, DispatchableNetwork, Dispatcher, Error, IncrementalStn, MinimalNetwork, Schedule, Stn,
    Ticks, TimePoint, Verdict,
};

/// Largest number of time points a fuzzed STN gets.
const MAX_POINTS: usize = 8;

/// Budget for every check: the networks are small, so no check runs out.
const BUDGET: Budget = Budget::UNLIMITED;

/// A bound value: mostly small, sometimes extreme, to reach the overflow paths.
#[derive(Debug, Clone, Copy, Arbitrary)]
enum Tick {
    /// A small value.
    Small(i8),
    /// `i64::MIN`, whose negation overflows.
    Min,
    /// `i64::MAX`.
    Max,
    /// Any value.
    Any(i64),
}

impl Tick {
    /// The value as [`Ticks`].
    fn ticks(self) -> Ticks {
        Ticks(match self {
            Self::Small(v) => i64::from(v),
            Self::Min => i64::MIN,
            Self::Max => i64::MAX,
            Self::Any(v) => v,
        })
    }
}

/// One construction step; indices are reduced modulo the current point count.
#[derive(Debug, Clone, Copy, Arbitrary)]
enum Op {
    /// `add_time_point`, ignored once the network has [`MAX_POINTS`] points.
    Point,
    /// `add_constraint(from, to, lo, hi)`.
    Constraint(u8, u8, Option<Tick>, Option<Tick>),
}

/// A constraint step with its handles resolved against the current point count.
type Step = (TimePoint, TimePoint, Option<Ticks>, Option<Ticks>);

fuzz_target!(|ops: Vec<Op>| {
    let Some(stn) = build(&ops) else {
        return;
    };

    check_algorithms(&stn);
    replay_incremental(&ops);
});

/// Resolves a constraint step for a network of `n >= 1` points.
fn step(n: usize, u: u8, v: u8, lo: Option<Tick>, hi: Option<Tick>) -> Step {
    let tp = |i: u8| TimePoint(usize::from(i) % n);

    (tp(u), tp(v), lo.map(Tick::ticks), hi.map(Tick::ticks))
}

/// Whether one more point fits under [`MAX_POINTS`].
fn has_room(n: usize) -> bool {
    n < MAX_POINTS
}

/// Builds the STN of every step, or `None` if a constraint is rejected (`lo == i64::MIN`).
fn build(ops: &[Op]) -> Option<Stn> {
    let mut stn = Stn::new();
    stn.add_time_point().ok()?;

    for &op in ops {
        let n = stn.time_point_count();

        match op {
            Op::Point if has_room(n) => drop(stn.add_time_point().ok()?),
            Op::Point => {}
            Op::Constraint(u, v, lo, hi) => {
                let (from, to, lo, hi) = step(n, u, v, lo, hi);
                stn.add_constraint(from, to, lo, hi).ok()?;
            }
        }
    }

    Some(stn)
}

/// Bellman-Ford, Floyd-Warshall, P³C and the dispatchable filter agree on `stn`.
fn check_algorithms(stn: &Stn) {
    let Some(bf) = answer(stn.check_consistency(&BUDGET)) else {
        return;
    };

    if let Verdict::Consistent(schedule) = &bf {
        check_schedule(stn, schedule);
    }

    let consistent = matches!(bf, Verdict::Consistent(_));
    let minimal = answer(stn.minimal_network(&BUDGET));

    if let Some(fw) = &minimal {
        assert_eq!(
            consistent,
            matches!(fw, Verdict::Consistent(_)),
            "BF and FW disagree"
        );
    }

    if let Some(Verdict::Consistent(min)) = &minimal {
        check_chordal(stn, min);
    }

    if let Some(verdict) = answer(stn.dispatchable(&BUDGET)) {
        assert_eq!(
            consistent,
            matches!(verdict, Verdict::Consistent(_)),
            "dispatchable disagrees"
        );

        if let Verdict::Consistent(net) = &verdict {
            check_network(net);
        }
    }
}

/// P³C reaches a consistent verdict, and its bounds equal `min`'s on every pair it covers.
fn check_chordal(stn: &Stn, min: &MinimalNetwork) {
    let Some(verdict) = answer(stn.partial_path_consistency(&BUDGET)) else {
        return;
    };

    let Verdict::Consistent(chordal) = verdict else {
        panic!("P³C rejects a consistent network");
    };

    for (a, b) in chordal.pairs() {
        assert_eq!(
            chordal.upper(a, b),
            min.upper(a, b),
            "upper bound of {a:?} -> {b:?}"
        );
        assert_eq!(
            chordal.lower(a, b),
            min.lower(a, b),
            "lower bound of {a:?} -> {b:?}"
        );
    }
}

/// Every constraint of `stn` holds under `schedule`.
fn check_schedule(stn: &Stn, schedule: &Schedule) {
    for c in stn.constraints() {
        let time = |tp: TimePoint| i128::from(schedule.time(tp).expect("scheduled").0);
        let diff = time(c.to) - time(c.from);

        assert!(
            c.lo.is_none_or(|lo| diff >= i128::from(lo.0)),
            "{c:?} violated below"
        );
        assert!(
            c.hi.is_none_or(|hi| diff <= i128::from(hi.0)),
            "{c:?} violated above"
        );
    }
}

/// A dispatchable network loads into a dispatcher and round-trips.
fn check_network(net: &DispatchableNetwork) {
    let mut d = Dispatcher::<MAX_POINTS, 1024>::EMPTY;
    d.load(net.points(), net.edges(), net.waits())
        .expect("a consistent STN's network is valid");

    let bytes = net.encode().expect("small");
    assert_eq!(DispatchableNetwork::decode(&bytes).as_ref(), Ok(net));
    d.load_encoded(&bytes).expect("its encoding loads");
}

/// Replays `ops` into an [`IncrementalStn`] and checks it against a from-scratch [`Stn`] of the
/// accepted constraints after every step.
fn replay_incremental(ops: &[Op]) {
    let mut inc = IncrementalStn::new();
    let mut accepted = Stn::new();
    inc.add_time_point(&BUDGET).expect("one point fits");
    accepted.add_time_point().expect("one point fits");

    for &op in ops {
        let n = accepted.time_point_count();

        match op {
            Op::Point if has_room(n) => {
                inc.add_time_point(&BUDGET).expect("fits");
                accepted.add_time_point().expect("fits");
            }
            Op::Point => {}
            Op::Constraint(u, v, lo, hi) => {
                let Some(next) = offer(&mut inc, &accepted, step(n, u, v, lo, hi)) else {
                    return;
                };

                accepted = next;
            }
        }
    }
}

/// Offers one constraint to `inc`, checks the decision against Bellman-Ford and the matrix
/// against Floyd-Warshall, and returns the new accepted network (`None` on overflow).
fn offer(inc: &mut IncrementalStn, accepted: &Stn, (from, to, lo, hi): Step) -> Option<Stn> {
    let mut candidate = accepted.clone();
    candidate.add_constraint(from, to, lo, hi).ok()?;
    let expected = matches!(
        answer(candidate.check_consistency(&BUDGET))?,
        Verdict::Consistent(_)
    );
    let decision = answer(inc.add_constraint(from, to, lo, hi, &BUDGET))?;

    assert_eq!(
        expected,
        matches!(decision, Verdict::Consistent(_)),
        "incremental disagrees"
    );

    let next = if expected {
        candidate
    } else {
        accepted.clone()
    };
    let Verdict::Consistent(reference) = answer(next.minimal_network(&BUDGET))? else {
        panic!("accepted constraints must stay consistent");
    };

    assert_eq!(answer(inc.minimal_network())?, reference, "matrix differs");

    Some(next)
}

/// The verdict if the algorithm answered, `None` on overflow; panics on every other error,
/// which a small network under an unlimited budget must not produce.
fn answer<T>(result: Result<T, Error>) -> Option<T> {
    match result {
        Ok(verdict) => Some(verdict),
        Err(Error::Overflow) => None,
        Err(e) => panic!("algorithm failed: {e:?}"),
    }
}
