//! Dispatcher: arbitrary (possibly malformed) networks and arbitrary call sequences.
//!
//! Checked: no call panics, for `usize` and `u16` index types; both index types make the same
//! decisions; a successful `execute` was allowed by `can_execute` and records the time; a
//! rejected load leaves the dispatcher unchanged.

#![no_main]

use libfuzzer_sys::arbitrary::{self, Arbitrary};
use libfuzzer_sys::fuzz_target;
use temporal_control::{DispatchEdge, Dispatcher, Index, PointKind, Ticks, TimePoint, Wait};

/// Point capacity of the fuzzed dispatchers.
const N: usize = 8;
/// Entry capacity of the fuzzed dispatchers.
const E: usize = 32;

/// A tick value: mostly small, sometimes extreme, to reach the overflow paths.
#[derive(Debug, Clone, Copy, Arbitrary)]
enum Tick {
    /// A small value.
    Small(i8),
    /// `i64::MIN`.
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

/// One dispatcher call; point indices are reduced modulo the point count plus one, so that
/// one out-of-range handle stays reachable.
#[derive(Debug, Clone, Copy, Arbitrary)]
enum Call {
    /// `observe`.
    Observe(u8, Tick),
    /// `execute`.
    Execute(u8, Tick),
    /// `step` with up to one observation.
    Step(Option<u8>, Tick),
    /// `check_deadlines`.
    Deadlines(Tick),
    /// Load the same network again, in place.
    Reload,
    /// Load an empty network that is too large, which must change nothing.
    RejectedLoad,
}

/// A fuzzer-chosen network and call sequence.
#[derive(Debug, Arbitrary)]
struct Input {
    /// Points `(executable, activation, lo, hi)`.
    points: Vec<(bool, u8, Tick, Tick)>,
    /// Edges `(from, to, weight)`.
    edges: Vec<(u8, u8, Tick)>,
    /// Waits `(waiter, contingent, delay)`.
    waits: Vec<(u8, u8, Tick)>,
    /// Calls in order.
    calls: Vec<Call>,
}

/// The network of an [`Input`] as dispatcher input slices.
struct Network {
    /// Point kinds.
    points: Vec<PointKind>,
    /// Edges.
    edges: Vec<DispatchEdge>,
    /// Waits.
    waits: Vec<Wait>,
}

impl Network {
    /// Builds the network, reducing indices modulo `points + 1`.
    fn new(input: &Input) -> Self {
        let points: Vec<_> = input.points.iter().take(N).collect();
        let tp = |i: u8| TimePoint(usize::from(i) % (points.len() + 1));
        Self {
            points: points
                .iter()
                .map(|&&(exec, a, lo, hi)| {
                    if exec {
                        PointKind::Executable
                    } else {
                        PointKind::Contingent {
                            activation: tp(a),
                            lo: lo.ticks(),
                            hi: hi.ticks(),
                        }
                    }
                })
                .collect(),
            edges: input
                .edges
                .iter()
                .take(E)
                .map(|&(f, t, w)| DispatchEdge {
                    from: tp(f),
                    to: tp(t),
                    weight: w.ticks(),
                })
                .collect(),
            waits: input
                .waits
                .iter()
                .take(E)
                .map(|&(x, c, d)| Wait {
                    waiter: tp(x),
                    contingent: tp(c),
                    delay: d.ticks(),
                })
                .collect(),
        }
    }

    /// Loads the network into `d`.
    fn load<I: Index>(&self, d: &mut Dispatcher<N, E, I>) -> bool {
        d.load(&self.points, &self.edges, &self.waits).is_ok()
    }
}

fuzz_target!(|input: Input| {
    let net = Network::new(&input);
    let mut wide = Dispatcher::<N, E>::EMPTY;
    let mut narrow = Dispatcher::<N, E, u16>::EMPTY;

    if !net.load(&mut wide) {
        assert!(!net.load(&mut narrow));
        return;
    }

    assert!(net.load(&mut narrow));
    let modulus = net.points.len() + 1;

    for call in input.calls {
        let a = run(&mut wide, &net, call, modulus);
        let b = run(&mut narrow, &net, call, modulus);
        assert_eq!(a, b, "index types disagree on {call:?}");
    }
});

/// Applies `call` to `d` and returns a comparable summary of the outcome.
fn run<I: Index>(d: &mut Dispatcher<N, E, I>, net: &Network, call: Call, modulus: usize) -> String {
    let tp = |i: u8| TimePoint(usize::from(i) % modulus);
    let outcome = match call {
        Call::Observe(p, t) => format!("{:?}", d.observe(tp(p), t.ticks())),
        Call::Execute(p, t) => execute(d, tp(p), t.ticks()),
        Call::Step(p, t) => {
            let observed: Vec<TimePoint> = p.map(tp).into_iter().collect();
            let mut out = [TimePoint(0); N];
            let r = d.step(t.ticks(), &observed, &mut out);
            let k = r.unwrap_or(0);
            format!("{r:?} {:?}", &out[..k])
        }
        Call::Deadlines(t) => format!("{:?}", d.check_deadlines(t.ticks())),
        Call::Reload => format!("{}", net.load(d)),
        Call::RejectedLoad => {
            let before = format!("{d:?}");
            assert!(d.load(&[PointKind::Executable; N + 1], &[], &[]).is_err());
            assert_eq!(
                format!("{d:?}"),
                before,
                "rejected load changed the dispatcher"
            );
            String::new()
        }
    };
    let probe = (0..modulus)
        .map(|p| (d.time(TimePoint(p)), d.window(TimePoint(p))))
        .collect::<Vec<_>>();

    format!(
        "{outcome} {probe:?} {:?} {}",
        d.next_wakeup(),
        d.is_complete()
    )
}

/// `execute`, checking that success means `can_execute` held and the time was recorded.
fn execute<I: Index>(d: &mut Dispatcher<N, E, I>, tp: TimePoint, now: Ticks) -> String {
    let allowed = d.can_execute(tp, now);
    let r = d.execute(tp, now);
    assert_eq!(
        allowed,
        r != Err(temporal_control::DispatchError::NotExecutable(tp))
    );

    if allowed {
        assert_eq!(d.time(tp), Some(now));
    }

    format!("{r:?}")
}
