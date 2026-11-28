# temporal-control

[![crates.io](https://img.shields.io/crates/v/temporal-control.svg)](https://crates.io/crates/temporal-control)
[![docs.rs](https://img.shields.io/docsrs/temporal-control)](https://docs.rs/temporal-control)
[![CI](https://github.com/sunsided/temporal-control/actions/workflows/ci.yml/badge.svg)](https://github.com/sunsided/temporal-control/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/sunsided/temporal-control/graph/badge.svg)](https://codecov.io/gh/sunsided/temporal-control)
[![MSRV](https://img.shields.io/crates/msrv/temporal-control)](https://crates.io/crates/temporal-control)
[![License](https://img.shields.io/crates/l/temporal-control)](https://github.com/sunsided/temporal-control#license)
[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Simple temporal networks with uncertainty: dynamic controllability checking on the ground and
dispatchable execution onboard, down to `no_std` targets without a heap.

A simple temporal network (STN) is a set of time points with "at least `a`, at most `b` ticks
apart" constraints between them; an STNU adds contingent links, durations that nature picks
within known bounds (an activity whose end the system only observes). The
crate decides whether such a network can always be executed, whatever the contingent durations
turn out to be, and if so compiles it into a dispatchable form that a small fixed-capacity
dispatcher executes tick by tick. When a network is not controllable, the answer names the
requirements and contingent bounds that conflict. All time values are integer ticks, all
arithmetic is checked, and every algorithm runs under an explicit compute and memory budget, so
an exhausted budget yields an error rather than a verdict.

Part of the [planning and search crates](https://sunsided.github.io/planning-and-search/); see also this crate's [overview page](https://sunsided.github.io/planning-and-search/crates/temporal-control/).

## Who this is for

- Flight software and robotics engineers who schedule activities with uncertain durations on a
  spacecraft, rover or robot and need the timing checked before uplink and executed onboard
  without a heap. The design targets adoption by an existing executive such as F Prime or cFS.
- Teams of vehicles (rovers, landers, orbiters) that share one plan but rarely talk: each
  gets a part it can execute and change alone, and the parts are merged and rebalanced when
  they can communicate, without a central solver.
- Planners that emit temporal plans and need to know whether the plan stays executable under
  duration uncertainty, with an explanation when it does not.
- Researchers comparing dynamic controllability algorithms: Morris 2014, RUL⁻, RUL2021, the
  labelled closure and a per-label backpropagation run on the same networks and are
  cross-checked against each other and a brute-force oracle.
- For energy, memory, thermal or communication budgets over the same timeline,
  [resource-timeline](https://github.com/sunsided/resource-timeline) is the companion crate;
  for durations with probabilities rather than bounds,
  [stochastic-search](https://github.com/sunsided/stochastic-search) is the better fit.

## Installation

```sh
cargo add temporal-control
```

For a target without a heap, keep only the onboard dispatcher and decoder:

```toml
[dependencies]
temporal-control = { version = "0.1", default-features = false }
```

## Scope

- STN consistency (Bellman-Ford) with negative-cycle explanation.
- STN minimal network (Floyd-Warshall).
- STN minimal constraints of the constrained pairs only (P³C, Planken, de Weerdt and van der
  Krogt 2008).
- Incremental STN consistency, `O(n^2)` per constraint.
- Dispatchable STN by dominated-edge filtering (Muscettola, Morris and Tsamardinos 1998, as
  restated by Morris 2014; minimal except for edge copies on simultaneous points, if that
  restatement is complete).
- Extended dispatchable STNU: Morris 2014 dispatchable variant, reduced by `minDispESTNU`
  (without diamonds).
- STNU dynamic controllability: labelled closure with extended dispatchable network
  (reference), Morris 2014 `O(n^3)` backpropagation (default checker), RUL⁻ (Cairo, Hunsberger
  and Rizzi 2018), RUL2021 (Hunsberger and Posenato 2022), and per-label backpropagation
  (cross-check).
- Explanations of non-controllability (semi-reducible negative cycles).
- Multi-agent STNs: temporal decoupling into per-agent local networks that execute
  independently (greedy cuts relative to a shared reference point), local changes checked
  by the agent alone, distributed consistency checks and re-decoupling from exchanged
  summaries of each agent's shared points, and communication windows (relay passes) modelled
  as contingent links.
- Onboard dispatcher, `no_std` without `alloc`, fixed capacity, loaded in place from a
  `const EMPTY`, with `u16`/`u32`/`usize` index types for its internal handles.
- Compact checksummed encoding (CRC-32) of dispatchable networks, with a no-`alloc` decoder.
- `cargo fuzz` targets (decoder, dispatcher calls, DC checker agreement, STN algorithm
  agreement) and Kani harnesses for the dispatcher and the decoder, all verified at small
  bounds (see section 8 of
  [docs/design.md](https://github.com/sunsided/temporal-control/blob/main/docs/design.md)).
- The papers' own examples as tests, and every algorithm compared with its paper (section 12
  of the design).

Reaction-time semantics: an observation at tick `t` may trigger executions at the same tick.
Next-tick reaction is supported by transforming the network (`Stnu::with_reaction`).

Not yet implemented: differential tests against CSTNU-Tool, and optimal (flexibility-maximising)
decoupling.

## Cargo features

| Feature | Default | Enables |
| --- | --- | --- |
| `alloc` | yes | Ground-side algorithms that need a heap: STN and STNU construction, consistency and controllability checking, minimal and dispatchable networks, the encoder |

Without `alloc`, the crate offers the `Dispatcher`, the decoder and their input types, and
builds for bare-metal targets such as `thumbv7em-none-eabihf` (checked in CI).

## MSRV

The library builds on Rust 1.85 (`rust-version` in `Cargo.toml`, checked in CI). Tests and
benches may need a newer toolchain. Raising the MSRV is a minor-version change.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](https://github.com/sunsided/temporal-control/blob/main/LICENSE-APACHE))
- MIT license ([LICENSE-MIT](https://github.com/sunsided/temporal-control/blob/main/LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion
in this work by you, as defined in the Apache-2.0 license, shall be dual licensed as above,
without any additional terms or conditions.

## Contributing

See [CONTRIBUTING.md](https://github.com/sunsided/temporal-control/blob/main/CONTRIBUTING.md)
for the setup, the Taskfile commands and the branch and commit conventions.
