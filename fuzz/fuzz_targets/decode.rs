//! Decoder: arbitrary bytes, and arbitrary records behind a valid header and checksum.
//!
//! Raw bytes almost never pass the CRC, so they only exercise the framing checks. In the
//! second mode the harness writes a correct header and CRC around fuzzer-chosen records, so
//! the structural validator and the in-place fill are reached. Checked: no panic; the
//! onboard decoder (`usize` and `u16` dispatchers) accepts exactly what the ground-side
//! `DispatchableNetwork::decode` accepts; accepted networks re-encode to the same bytes and
//! can be dispatched without panicking.

#![no_main]

use libfuzzer_sys::fuzz_target;
use temporal_control::dispatch::encoding::{VERSION, crc32};
use temporal_control::{DispatchableNetwork, Dispatcher, Ticks, TimePoint};

/// Point capacity of the fuzzed dispatchers; the framed mode never exceeds it.
const N: usize = 16;
/// Entry capacity of the fuzzed dispatchers; the framed mode never exceeds it.
const E: usize = 64;

fuzz_target!(|data: &[u8]| {
    match data.split_first() {
        Some((&mode, rest)) if mode % 2 == 1 => check(&framed(rest)),
        _ => check(data),
    }
});

/// Builds an encoding with a correct header and CRC: three count bytes (reduced so the
/// network fits [`N`] and [`E`]), then the records, zero-padded or cut to the right length.
fn framed(data: &[u8]) -> Vec<u8> {
    let count = |i: usize, max: u8| u32::from(data.get(i).copied().unwrap_or(0) % max);
    let (n, e, w) = (count(0, 16), count(1, 16), count(2, 16));
    let len = usize::try_from(21 * n + 16 * (e + w)).expect("small");
    let mut body: Vec<u8> = data.iter().skip(3).copied().take(len).collect();
    body.resize(len, 0);

    let mut bytes = b"TCDN".to_vec();
    bytes.extend(VERSION.to_le_bytes());
    bytes.extend(0u16.to_le_bytes());

    for c in [n, e, w] {
        bytes.extend(c.to_le_bytes());
    }

    bytes.extend(body);
    bytes.extend(crc32(&bytes).to_le_bytes());
    bytes
}

/// Decodes `bytes` every way and checks that the decoders agree.
fn check(bytes: &[u8]) {
    let decoded = DispatchableNetwork::decode(bytes);
    let mut wide = Dispatcher::<N, E>::EMPTY;
    let mut narrow = Dispatcher::<N, E, u16>::EMPTY;
    let loaded = [
        wide.load_encoded(bytes).is_ok(),
        narrow.load_encoded(bytes).is_ok(),
    ];

    let Ok(net) = decoded else {
        assert_eq!(
            loaded,
            [false, false],
            "onboard accepted what decode rejected"
        );
        return;
    };

    assert_eq!(net.encode().expect("decoded networks are small"), bytes);
    let fits = net.points().len() <= N && net.entry_count() <= E;
    assert_eq!(loaded, [fits, fits], "onboard and decode disagree");

    let mut out = [TimePoint(0); N];

    for t in -1..40 {
        let _ = wide.step(Ticks(t), &[], &mut out);
        let _ = narrow.step(Ticks(t), &[], &mut out);
    }
}
