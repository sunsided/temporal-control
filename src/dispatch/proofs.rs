//! Bounded no-panic checks of the dispatcher and the decoder on small fixed capacities.
//!
//! The checks are ordinary functions over plain-data scenarios. Unit tests run them on
//! pseudo-random concrete scenarios; under `cfg(kani)` the harnesses in [`harnesses`] run them
//! on fully symbolic ones, which proves the absence of panics (including arithmetic overflow
//! and out-of-bounds indexing) for every input of that size. See `docs/design.md`, section 8,
//! for how to run Kani.

use super::encoding::{VERSION, crc32};
use super::{DispatchEdge, DispatchError, Dispatcher, Index, PointKind, Wait};
use crate::{Ticks, TimePoint};

/// Point capacity of the checked dispatchers.
const N: usize = 3;

/// Edge slots of a scenario.
const EDGES: usize = 3;

/// Entry capacity of the checked dispatchers: every edge slot plus one wait.
const E: usize = 2 * EDGES + 1;

/// Calls per scenario.
const CALLS: usize = 3;

/// Length of a framed encoding with one point and one edge.
const FRAMED_LEN: usize = 24 + 21 + 16;

/// A network and a call sequence, as raw values; indices are reduced modulo `N + 1`, so one
/// out-of-range handle is always reachable.
#[derive(Debug, Clone, Copy)]
pub(super) struct Scenario {
    /// Points `(executable, activation, lo, hi)`; the first `point_count % (N + 1)` are used.
    pub(super) points: [(bool, u8, i64, i64); N],
    /// Number of points used, reduced modulo `N + 1`.
    pub(super) point_count: u8,
    /// Edges `(from, to, weight)`; the first `edge_count % (EDGES + 1)` are used.
    pub(super) edges: [(u8, u8, i64); EDGES],
    /// Number of edges used, reduced modulo `EDGES + 1`.
    pub(super) edge_count: u8,
    /// One optional wait `(waiter, contingent, delay)`.
    pub(super) wait: Option<(u8, u8, i64)>,
    /// Calls `(selector, point, now)`; `selector % 5` picks observe, execute, `can_execute`,
    /// step or `check_deadlines`.
    pub(super) calls: [(u8, u8, i64); CALLS],
}

/// A handle from a raw value, reduced modulo `N + 1`.
fn handle(raw: u8) -> TimePoint {
    TimePoint(usize::from(raw) % (N + 1))
}

/// Loads the scenario's network (if valid) and runs its calls, asserting that executions
/// only happen inside the window and are recorded. Panics only if an assertion fails.
pub(super) fn run_scenario<I: Index>(s: &Scenario) {
    let points = s.points.map(|(exec, a, lo, hi)| {
        if exec {
            PointKind::Executable
        } else {
            PointKind::Contingent {
                activation: handle(a),
                lo: Ticks(lo),
                hi: Ticks(hi),
            }
        }
    });
    let edges = s.edges.map(|(f, t, w)| DispatchEdge {
        from: handle(f),
        to: handle(t),
        weight: Ticks(w),
    });
    let waits = s.wait.map(|(x, c, d)| Wait {
        waiter: handle(x),
        contingent: handle(c),
        delay: Ticks(d),
    });
    let n = usize::from(s.point_count) % (N + 1);
    let m = usize::from(s.edge_count) % (EDGES + 1);

    let mut d = Dispatcher::<N, E, I>::EMPTY;

    if d.load(&points[..n], &edges[..m], waits.as_slice()).is_err() {
        return;
    }

    for (selector, p, now) in s.calls {
        call(&mut d, selector, handle(p), Ticks(now));
    }
}

/// Runs one dispatcher call and checks its postconditions.
fn call<I: Index>(d: &mut Dispatcher<N, E, I>, selector: u8, tp: TimePoint, now: Ticks) {
    match selector % 5 {
        0 => drop(d.observe(tp, now)),
        1 => execute(d, tp, now),
        2 => drop(d.can_execute(tp, now)),
        3 => drop(d.step(now, &[tp], &mut [TimePoint(0); N])),
        _ => drop(d.check_deadlines(now)),
    }

    let _ = (d.next_wakeup(), d.is_complete(), d.window(tp), d.time(tp));
}

/// `execute`, asserting that a point only executes inside its window and is then recorded.
fn execute<I: Index>(d: &mut Dispatcher<N, E, I>, tp: TimePoint, now: Ticks) {
    let window = d.window(tp);
    let result = d.execute(tp, now);

    if result == Err(DispatchError::NotExecutable(tp)) {
        return;
    }

    let (lb, ub) = window.expect("only known points execute");
    assert!(lb.is_none_or(|l| l <= now) && ub.is_none_or(|u| now <= u));
    assert_eq!(d.time(tp), Some(now));
}

/// Feeds `bytes` to the decoder; if accepted, runs a few ticks.
pub(super) fn decode_bytes(bytes: &[u8]) {
    let mut d = Dispatcher::<2, 4, u16>::EMPTY;

    if d.load_encoded(bytes).is_ok() {
        for t in 0..3 {
            let _ = d.step(Ticks(t), &[], &mut [TimePoint(0); 2]);
        }
    }
}

/// Wraps one point record and one edge record in a correct header and checksum, so that the
/// decoder gets past the framing checks.
pub(super) fn framed(point: [u8; 21], edge: [u8; 16]) -> [u8; FRAMED_LEN] {
    let mut bytes = [0u8; FRAMED_LEN];
    bytes[..4].copy_from_slice(b"TCDN");
    bytes[4..6].copy_from_slice(&VERSION.to_le_bytes());
    bytes[8..12].copy_from_slice(&1u32.to_le_bytes());
    bytes[12..16].copy_from_slice(&1u32.to_le_bytes());
    bytes[20..41].copy_from_slice(&point);
    bytes[41..57].copy_from_slice(&edge);
    let crc = crc32(&bytes[..57]);
    bytes[57..].copy_from_slice(&crc.to_le_bytes());
    bytes
}

#[cfg(kani)]
mod harnesses {
    //! Kani proof harnesses: the checks above on symbolic inputs.

    use super::{FRAMED_LEN, Scenario, decode_bytes, framed, run_scenario};

    /// A fully symbolic scenario.
    fn any_scenario() -> Scenario {
        Scenario {
            points: kani::any(),
            point_count: kani::any(),
            edges: kani::any(),
            edge_count: kani::any(),
            wait: kani::any(),
            calls: kani::any(),
        }
    }

    /// Loading and `step`/`execute`/`can_execute`/`observe`/`check_deadlines` never panic
    /// (`u16` indices), and executions stay inside their windows.
    #[kani::proof]
    #[kani::unwind(9)]
    fn dispatcher_never_panics_u16() {
        run_scenario::<u16>(&any_scenario());
    }

    /// As [`dispatcher_never_panics_u16`] with the default `usize` indices.
    #[kani::proof]
    #[kani::unwind(9)]
    fn dispatcher_never_panics_usize() {
        run_scenario::<usize>(&any_scenario());
    }

    /// Arbitrary bytes up to the framed length never panic the decoder (framing checks).
    #[kani::proof]
    #[kani::unwind(66)]
    fn decoder_never_panics_on_raw_bytes() {
        let bytes: [u8; FRAMED_LEN] = kani::any();
        let len: usize = kani::any();
        kani::assume(len <= FRAMED_LEN);
        decode_bytes(&bytes[..len]);
    }

    /// Arbitrary records behind a valid header and checksum never panic the decoder
    /// (structural validation and the in-place fill).
    #[kani::proof]
    #[kani::unwind(66)]
    fn decoder_never_panics_on_framed_records() {
        decode_bytes(&framed(kani::any(), kani::any()));
    }
}

#[cfg(test)]
mod tests {
    //! The Kani checks on pseudo-random concrete inputs, so they also run without Kani.

    use super::{EDGES, N, Scenario, decode_bytes, framed, run_scenario};

    /// Small xorshift generator; deterministic so failures replay.
    struct Rng(u64);

    impl Rng {
        /// Next pseudo-random value.
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        /// A pseudo-random byte.
        fn byte(&mut self) -> u8 {
            self.next().to_le_bytes()[0]
        }

        /// Mostly zero, else a value below 4: keeps tags and indices valid often enough to
        /// reach the structural checks.
        fn sparse_byte(&mut self) -> u8 {
            if self.next() % 8 == 0 {
                self.byte() % 4
            } else {
                0
            }
        }

        /// A tick value: small most of the time, sometimes extreme or arbitrary.
        fn tick(&mut self) -> i64 {
            match self.next() % 8 {
                0 => i64::MIN,
                1 => i64::MAX,
                2 => i64::from_le_bytes(self.next().to_le_bytes()),
                _ => i64::from(self.byte() % 16) - 6,
            }
        }

        /// A pseudo-random scenario.
        fn scenario(&mut self) -> Scenario {
            Scenario {
                points: [(); N]
                    .map(|()| (self.next() % 2 == 0, self.byte(), self.tick(), self.tick())),
                point_count: self.byte(),
                edges: [(); EDGES].map(|()| (self.byte(), self.byte(), self.tick())),
                edge_count: self.byte(),
                wait: (self.next() % 2 == 0).then(|| (self.byte(), self.byte(), self.tick())),
                calls: [(); 3].map(|()| (self.byte(), self.byte(), self.tick())),
            }
        }
    }

    /// The dispatcher check on many concrete scenarios.
    #[test]
    fn dispatcher_scenarios() {
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);

        for _ in 0..20_000 {
            let s = rng.scenario();
            run_scenario::<u16>(&s);
            run_scenario::<usize>(&s);
        }
    }

    /// The decoder checks on many concrete byte strings, raw and framed.
    #[test]
    fn decoder_inputs() {
        let mut rng = Rng(0xD1B5_4A32_D192_ED03);

        for _ in 0..20_000 {
            let point = [(); 21].map(|()| rng.sparse_byte());
            let edge = [(); 16].map(|()| rng.sparse_byte());
            let bytes = framed(point, edge);
            decode_bytes(&bytes);
            decode_bytes(&bytes[..usize::from(rng.byte()) % bytes.len()]);
        }
    }
}
