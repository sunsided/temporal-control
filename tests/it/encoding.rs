//! Compact encoding: round trips, corruption detection and robustness of the decoder.

use proptest::prelude::*;
use temporal_control::dispatch::encoding::{VERSION, crc32, encode_into, encoded_len};
use temporal_control::{
    Budget, Controllability, DecodeError, DispatchError, DispatchableNetwork, Dispatcher, Stn,
    Stnu, Ticks, TimePoint, Verdict,
};

use crate::generators::any_stn;
use crate::sim::{self, Agent, E, N, Nature, Rng};
use crate::stnu_gen::medium_stnu;

/// Budget for generated STNUs; a hang fails the test instead of stalling it.
const TEST_BUDGET: Budget = Budget {
    max_steps: 50_000_000,
    max_cells: 1 << 20,
};

/// The extended dispatchable network of `stnu`, or `None` if it is not DC.
fn estnu(stnu: &Stnu) -> Option<DispatchableNetwork> {
    match stnu.dispatchable(&TEST_BUDGET).unwrap() {
        Controllability::Controllable(net) => Some(net),
        Controllability::Uncontrollable(_) => None,
    }
}

/// The dispatchable network of `stn`, or `None` if it is inconsistent.
fn dispatchable_stn(stn: &Stn) -> Option<DispatchableNetwork> {
    match stn.dispatchable(&Budget::UNLIMITED).unwrap() {
        Verdict::Consistent(net) => Some(net),
        Verdict::Inconsistent(_) => None,
    }
}

/// A DC network with a contingent link, a wait and a deadline: `A => C [2, 8]`, `X` within
/// `[0, 3]` after `C`, and `X` at most 9 after `A`.
fn sample() -> DispatchableNetwork {
    let mut stnu = Stnu::new();
    let a = stnu.add_time_point().unwrap();
    let c = stnu.add_contingent_link(a, Ticks(2), Ticks(8)).unwrap();
    let x = stnu.add_time_point().unwrap();
    stnu.add_constraint(c, x, Some(Ticks(0)), Some(Ticks(3)))
        .unwrap();
    stnu.add_constraint(a, x, Some(Ticks(0)), Some(Ticks(9)))
        .unwrap();
    estnu(&stnu).expect("sample is DC")
}

/// A dispatcher that is already executing `net`, to show that rejected loads keep it.
fn busy(net: &DispatchableNetwork) -> Dispatcher<N, E> {
    let mut d: Dispatcher<N, E> = net.dispatcher().unwrap();
    d.step(Ticks(0), &[], &mut [TimePoint(0); N]).unwrap();
    d
}

/// Header plus records `body` plus a correct checksum, with the given counts.
fn seal(counts: [u32; 3], reserved: u16, body: &[u8]) -> Vec<u8> {
    let mut bytes = b"TCDN".to_vec();
    bytes.extend(VERSION.to_le_bytes());
    bytes.extend(reserved.to_le_bytes());

    for c in counts {
        bytes.extend(c.to_le_bytes());
    }

    bytes.extend(body);
    bytes.extend(crc32(&bytes).to_le_bytes());
    bytes
}

/// Asserts that `net` survives encoding unchanged and dispatches exactly like the original.
fn assert_round_trip(net: &DispatchableNetwork, seed: u64) {
    let bytes = net.encode().unwrap();
    assert_eq!(
        Some(bytes.len()),
        encoded_len(net.points().len(), net.edges().len(), net.waits().len())
    );
    assert_eq!(DispatchableNetwork::decode(&bytes).as_ref(), Ok(net));

    for agent in [Agent::Asap, Agent::Latest, Agent::Random] {
        let expected = sim::run(net, agent, Nature::Random, &mut Rng(seed));
        let mut wide = Dispatcher::<N, E>::EMPTY;
        wide.load_encoded(&bytes).unwrap();
        let mut narrow = Dispatcher::<N, E, u16>::EMPTY;
        narrow.load_encoded(&bytes).unwrap();
        let runs = [
            sim::run_on(&mut wide, net, agent, Nature::Random, &mut Rng(seed)),
            sim::run_on(&mut narrow, net, agent, Nature::Random, &mut Rng(seed)),
        ];
        assert_eq!(runs, [expected.clone(), expected]);
    }
}

/// The hand-written sample round-trips and its encoding has the documented size.
#[test]
fn sample_round_trips() {
    let net = sample();
    assert_ne!(net.waits(), []);
    assert_round_trip(&net, 3);

    let bytes = net.encode().unwrap();
    let mut short = vec![0; bytes.len() - 1];
    assert_eq!(
        encode_into(net.points(), net.edges(), net.waits(), &mut short),
        Err(temporal_control::EncodeError::BufferTooSmall {
            needed: bytes.len()
        })
    );
}

/// Every single-bit flip of the sample encoding is rejected and leaves the dispatcher as it
/// was.
#[test]
fn every_bit_flip_is_rejected() {
    let net = sample();
    let bytes = net.encode().unwrap();
    let mut d = busy(&net);
    let before = format!("{d:?}");

    for bit in 0..bytes.len() * 8 {
        let mut flipped = bytes.clone();
        flipped[bit / 8] ^= 1 << (bit % 8);
        assert!(d.load_encoded(&flipped).is_err(), "bit {bit} accepted");
        assert!(DispatchableNetwork::decode(&flipped).is_err(), "bit {bit}");
    }

    assert_eq!(format!("{d:?}"), before);
}

/// Every proper prefix, and the encoding with a trailing byte, is rejected.
#[test]
fn every_truncation_is_rejected() {
    let net = sample();
    let bytes = net.encode().unwrap();
    let mut d = busy(&net);
    let before = format!("{d:?}");

    for len in 0..bytes.len() {
        assert!(
            d.load_encoded(&bytes[..len]).is_err(),
            "prefix {len} accepted"
        );
    }

    let mut longer = bytes.clone();
    longer.push(0);
    assert!(matches!(
        d.load_encoded(&longer),
        Err(DecodeError::LengthMismatch { .. })
    ));
    assert_eq!(format!("{d:?}"), before);
}

/// Each framing check reports its own error, in the documented order.
#[test]
fn decode_errors_are_typed() {
    let exec = [0u8; 21];
    let mut d = Dispatcher::<2, 4>::EMPTY;
    let mut check = |bytes: &[u8], expected: DecodeError| {
        assert_eq!(d.load_encoded(bytes), Err(expected));
    };

    check(&[], DecodeError::Truncated);
    let ok = seal([1, 0, 0], 0, &exec);
    let mut magic = ok.clone();
    magic[0] = b'X';
    check(&magic, DecodeError::BadMagic);
    let mut version = ok.clone();
    version[4] = 2;
    check(&version, DecodeError::UnsupportedVersion(2));
    check(
        &seal([2, 0, 0], 0, &exec),
        DecodeError::LengthMismatch {
            expected: 66,
            actual: 45,
        },
    );
    let mut crc = ok.clone();
    *crc.last_mut().unwrap() ^= 1;
    check(&crc, DecodeError::ChecksumMismatch);
    check(&seal([1, 0, 0], 1, &exec), DecodeError::NonCanonical);
    let mut padded = exec;
    padded[7] = 1;
    check(&seal([1, 0, 0], 0, &padded), DecodeError::NonCanonical);
    let mut tag = exec;
    tag[0] = 2;
    check(&seal([1, 0, 0], 0, &tag), DecodeError::NonCanonical);

    let mut edge = 0u32.to_le_bytes().to_vec();
    edge.extend(5u32.to_le_bytes());
    edge.extend(1i64.to_le_bytes());
    check(
        &seal([1, 1, 0], 0, &[&exec[..], &edge].concat()),
        DecodeError::Network(DispatchError::InvalidNetwork),
    );
    check(
        &seal([3, 0, 0], 0, &[exec; 3].concat()),
        DecodeError::Network(DispatchError::CapacityExceeded),
    );
    assert_eq!(d.load_encoded(&ok), Ok(()));
}

/// Point records `(tag, activation, lo, hi)` and edge or wait records `(a, b, ticks)` over
/// `n` points, mostly well-formed: tags are rarely unknown and indices rarely out of range.
#[expect(
    clippy::type_complexity,
    reason = "plain record tuples of a test strategy"
)]
fn framed_records(
    n: u32,
) -> (
    impl Strategy<Value = Vec<(u8, u32, i64, i64)>>,
    impl Strategy<Value = Vec<(u32, u32, i64)>>,
) {
    let tag = prop_oneof![8 => Just(0u8), 4 => Just(1u8), 1 => Just(2u8)];
    let point = (tag, 0..=n, -1i64..4, -1i64..6);
    let usize_n = usize::try_from(n).unwrap();
    (
        proptest::collection::vec(point, usize_n..=usize_n),
        proptest::collection::vec((0..=n, 0..=n, -6i64..9), 0..8),
    )
}

proptest! {
    /// Dispatchable STNs round-trip and dispatch identically after decoding.
    #[test]
    fn stn_networks_round_trip(spec in any_stn(), seed in 1u64..) {
        if let Some(net) = dispatchable_stn(&spec.build().unwrap()) {
            assert_round_trip(&net, seed);
        }
    }

    /// Extended dispatchable STNUs (with waits) round-trip and dispatch identically.
    #[test]
    fn stnu_networks_round_trip(spec in medium_stnu(), seed in 1u64..) {
        if let Some(net) = estnu(&spec.build()) {
            assert_round_trip(&net, seed);
        }
    }

    /// Arbitrary bytes never panic the decoder and are (almost surely) rejected.
    #[test]
    fn random_bytes_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..200)) {
        let mut d = Dispatcher::<8, 32>::EMPTY;
        let _ = d.load_encoded(&bytes);
        let _ = DispatchableNetwork::decode(&bytes);
    }

    /// Arbitrary records behind a valid header and checksum reach the structural validator;
    /// it never panics, the dispatcher accepts what [`DispatchableNetwork::decode`] accepts
    /// (when it fits), and accepted networks re-encode to the same bytes.
    #[test]
    fn framed_random_records_never_panic(
        (points, pairs) in (1u32..6).prop_flat_map(framed_records),
        waits in 0usize..3,
        junk in proptest::option::weighted(0.2, (0usize..256, any::<u8>())),
    ) {
        let mut body = Vec::new();

        for &(tag, activation, lo, hi) in &points {
            body.push(tag);

            if tag == 0 {
                body.extend([0; 20]);
            } else {
                body.extend(activation.to_le_bytes());
                body.extend(lo.to_le_bytes());
                body.extend(hi.to_le_bytes());
            }
        }

        for &(a, b, t) in &pairs {
            body.extend(a.to_le_bytes());
            body.extend(b.to_le_bytes());
            body.extend(t.to_le_bytes());
        }

        if let Some((at, byte)) = junk {
            if let Some(slot) = body.get_mut(at) {
                *slot = byte;
            }
        }

        let waits = waits.min(pairs.len());
        let count = |c: usize| u32::try_from(c).unwrap();
        let counts = [count(points.len()), count(pairs.len() - waits), count(waits)];
        let bytes = seal(counts, 0, &body);

        let mut d = Dispatcher::<8, 32, u16>::EMPTY;
        let loaded = d.load_encoded(&bytes);
        let decoded = DispatchableNetwork::decode(&bytes);
        prop_assert_eq!(loaded.is_ok(), decoded.is_ok(), "{:?} {:?}", loaded, decoded);

        if let Ok(net) = decoded {
            prop_assert_eq!(net.encode().unwrap(), bytes);
            let mut out = [TimePoint(0); 8];

            for t in -2..20 {
                let _ = d.step(Ticks(t), &[], &mut out);
            }
        }
    }
}
