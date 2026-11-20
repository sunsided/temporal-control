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
const N: usize = 2;

/// Edge slots of a scenario. One: with two, a single-call harness already peaked at 12 GB.
const EDGES: usize = 1;

/// Entry capacity of the checked dispatchers: every edge slot plus one wait.
const E: usize = 2 * EDGES + 1;

/// Length of a framed encoding with one point record and nothing else.
const FRAMED_LEN: usize = 24 + 21;

/// Length of the header plus checksum, the shortest acceptable encoding (an empty network).
#[cfg(kani)]
const EMPTY_LEN: usize = 24;

/// Length of an encoding with one point record and one edge record.
#[cfg(kani)]
const POINT_EDGE_LEN: usize = 24 + 21 + 16;

/// A network as raw values; indices are reduced modulo `N + 1`, so one out-of-range handle is
/// always reachable.
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
}

/// One dispatcher call `(selector, point, now)`; `selector % 5` picks observe, execute,
/// `can_execute`, step or `check_deadlines`.
pub(super) type Call = (u8, u8, i64);

/// A handle from a raw value, reduced modulo `N + 1`.
fn handle(raw: u8) -> TimePoint {
    TimePoint(usize::from(raw) % (N + 1))
}

/// Loads the scenario's network (if valid) and runs `calls`, asserting that executions only
/// happen inside the window and are recorded. Panics only if an assertion fails.
pub(super) fn run_scenario<I: Index>(s: &Scenario, calls: &[Call]) {
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

    for &(selector, p, now) in calls {
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
        run_ticks(&mut d, 3);
    }
}

/// Runs `ticks` steps without observations from tick 0 on.
fn run_ticks(d: &mut Dispatcher<2, 4, u16>, ticks: i64) {
    for t in 0..ticks {
        let _ = d.step(Ticks(t), &[], &mut [TimePoint(0); 2]);
    }
}

/// Loads `points` point and `edges` edge records from `bytes` as if the framing and
/// canonical-form checks had passed: the structural validation and the in-place fill. What a
/// loaded dispatcher does next is covered by the dispatcher harnesses, which load the same
/// state from slices.
#[cfg(kani)]
fn load_records(bytes: &[u8], points: usize, edges: usize) {
    let encoded = super::encoding::Encoded::assume_valid_framing(bytes, points, edges);
    let _ = Dispatcher::<1, 2, u16>::EMPTY.load_from(&encoded);
}

/// A symbolic point record in canonical form: an executable record is all zero.
#[cfg(kani)]
fn any_canonical_point() -> [u8; 21] {
    let mut record: [u8; 21] = kani::any();

    if record[0] != 1 {
        record = [0; 21];
    }

    record
}

/// A header (magic, version, the given counts) followed by zeroed records and checksum.
#[cfg(kani)]
fn header<const L: usize>(points: u32, edges: u32) -> [u8; L] {
    let mut bytes = [0u8; L];
    bytes[..4].copy_from_slice(b"TCDN");
    bytes[4..6].copy_from_slice(&VERSION.to_le_bytes());
    bytes[8..12].copy_from_slice(&points.to_le_bytes());
    bytes[12..16].copy_from_slice(&edges.to_le_bytes());
    bytes
}

/// Wraps one point record in a correct header and checksum, so that the decoder gets past
/// the framing checks.
pub(super) fn framed(point: [u8; 21]) -> [u8; FRAMED_LEN] {
    let mut bytes = [0u8; FRAMED_LEN];
    bytes[..4].copy_from_slice(b"TCDN");
    bytes[4..6].copy_from_slice(&VERSION.to_le_bytes());
    bytes[8..12].copy_from_slice(&1u32.to_le_bytes());
    bytes[20..41].copy_from_slice(&point);
    let crc = crc32(&bytes[..41]);
    bytes[41..].copy_from_slice(&crc.to_le_bytes());
    bytes
}

#[cfg(kani)]
mod harnesses {
    //! Kani proof harnesses: the checks above on symbolic inputs. Each dispatcher harness
    //! loads a symbolic network and makes one symbolic call of one kind, so that each stays
    //! small enough for CBMC. Two calls in one harness exceeded 12 GB; call sequences are
    //! left to the unit tests below and the `dispatch` fuzz target.

    use super::{
        Call, EMPTY_LEN, FRAMED_LEN, POINT_EDGE_LEN, Scenario, any_canonical_point, header,
        load_records, run_scenario,
    };

    /// A fully symbolic network.
    fn any_scenario() -> Scenario {
        Scenario {
            points: kani::any(),
            point_count: kani::any(),
            edges: kani::any(),
            edge_count: kani::any(),
            wait: kani::any(),
        }
    }

    /// A symbolic call whose selector picks `kind` (see [`Call`]).
    fn any_call(kind: u8) -> Call {
        (kind, kani::any(), kani::any())
    }

    /// Loading plus one `observe` never panics (`u16` indices).
    #[kani::proof]
    #[kani::unwind(4)]
    fn dispatcher_observe_u16() {
        run_scenario::<u16>(&any_scenario(), &[any_call(0)]);
    }

    /// Loading plus one `execute` never panics and executes only inside the window (`u16`).
    #[kani::proof]
    #[kani::unwind(4)]
    fn dispatcher_execute_u16() {
        run_scenario::<u16>(&any_scenario(), &[any_call(1)]);
    }

    /// Loading plus one `can_execute` never panics (`u16` indices).
    #[kani::proof]
    #[kani::unwind(4)]
    fn dispatcher_can_execute_u16() {
        run_scenario::<u16>(&any_scenario(), &[any_call(2)]);
    }

    /// Loading plus one `step` never panics (`u16` indices).
    #[kani::proof]
    #[kani::unwind(4)]
    fn dispatcher_step_u16() {
        run_scenario::<u16>(&any_scenario(), &[any_call(3)]);
    }

    /// Loading plus one `check_deadlines` never panics (`u16` indices).
    #[kani::proof]
    #[kani::unwind(4)]
    fn dispatcher_check_deadlines_u16() {
        run_scenario::<u16>(&any_scenario(), &[any_call(4)]);
    }

    /// Loading plus one `execute` with the default `usize` indices.
    #[kani::proof]
    #[kani::unwind(4)]
    fn dispatcher_execute_usize() {
        run_scenario::<usize>(&any_scenario(), &[any_call(1)]);
    }

    /// Any input shorter than a header plus checksum is rejected by the framing checks
    /// without a panic. Only `Encoded::parse` runs: [`Dispatcher::load_encoded`] loads
    /// nothing unless it succeeds, and running the whole loader on a symbolic length made
    /// CBMC explore every loop of it.
    #[kani::proof]
    #[kani::unwind(4)]
    fn decoder_short_input() {
        let bytes: [u8; EMPTY_LEN - 1] = kani::any();
        let len: usize = kani::any();
        kani::assume(len < EMPTY_LEN);
        assert!(super::super::encoding::Encoded::parse(&bytes[..len]).is_err());
    }

    /// A 24-byte input with zero counts and arbitrary magic, version, reserved field and
    /// checksum never panics the framing, checksum and canonical-form checks. Only
    /// `Encoded::parse` runs; loading the (then empty) network as well exceeded 12 GB.
    #[kani::proof]
    #[kani::unwind(22)]
    fn decoder_framing_and_checksum() {
        let mut bytes: [u8; EMPTY_LEN] = [0; EMPTY_LEN];
        let head: [u8; 8] = kani::any();
        let crc: [u8; 4] = kani::any();
        bytes[..8].copy_from_slice(&head);
        bytes[20..].copy_from_slice(&crc);
        let _ = super::super::encoding::Encoded::parse(&bytes);
    }

    /// One arbitrary point record behind a valid header never panics the length and
    /// canonical-form checks (checksum skipped).
    #[kani::proof]
    #[kani::unwind(22)]
    fn decoder_canonical_point_record() {
        let mut bytes: [u8; FRAMED_LEN] = header(1, 0);
        let point: [u8; 21] = kani::any();
        bytes[20..41].copy_from_slice(&point);
        let _ = super::super::encoding::Encoded::parse_without_checksum(&bytes);
    }

    /// Loading one canonical point record never panics the structural validation or the
    /// in-place fill.
    #[kani::proof]
    #[kani::unwind(4)]
    fn decoder_load_point_record() {
        let mut bytes: [u8; FRAMED_LEN] = header(1, 0);
        bytes[20..41].copy_from_slice(&any_canonical_point());
        load_records(&bytes, 1, 0);
    }

    /// Loading one canonical point record and one arbitrary edge record never panics.
    #[kani::proof]
    #[kani::unwind(4)]
    fn decoder_load_point_and_edge_records() {
        let mut bytes: [u8; POINT_EDGE_LEN] = header(1, 1);
        let edge: [u8; 16] = kani::any();
        bytes[20..41].copy_from_slice(&any_canonical_point());
        bytes[41..57].copy_from_slice(&edge);
        load_records(&bytes, 1, 1);
    }
}

#[cfg(test)]
mod tests {
    //! The Kani checks on pseudo-random concrete inputs, so they also run without Kani.

    use super::{Call, EDGES, N, Scenario, decode_bytes, framed, run_scenario};

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
            }
        }

        /// Three pseudo-random calls.
        fn calls(&mut self) -> [Call; 3] {
            [(); 3].map(|()| (self.byte(), self.byte(), self.tick()))
        }
    }

    /// The dispatcher check on many concrete scenarios.
    #[test]
    fn dispatcher_scenarios() {
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);

        for _ in 0..20_000 {
            let s = rng.scenario();
            let calls = rng.calls();
            run_scenario::<u16>(&s, &calls);
            run_scenario::<usize>(&s, &calls);
        }
    }

    /// The decoder checks on many concrete byte strings, raw and framed.
    #[test]
    fn decoder_inputs() {
        let mut rng = Rng(0xD1B5_4A32_D192_ED03);

        for _ in 0..20_000 {
            let point = [(); 21].map(|()| rng.sparse_byte());
            let bytes = framed(point);
            decode_bytes(&bytes);
            decode_bytes(&bytes[..usize::from(rng.byte()) % bytes.len()]);
        }
    }
}
