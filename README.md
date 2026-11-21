# temporal-control

Simple temporal networks with uncertainty: dynamic controllability checking and dispatchable execution for no_std targets.

## Scope and status

Pre-release (0.0.0) on `draft/initial`; the API will change. See [docs/design.md](docs/design.md).

| Feature | Status |
| --- | --- |
| STN consistency (Bellman-Ford) with negative-cycle explanation | done |
| STN minimal network (Floyd-Warshall) | done |
| STN minimal constraints of the constrained pairs only (P³C, Planken, de Weerdt and van der Krogt 2008) | done |
| Incremental STN consistency, `O(n^2)` per constraint | done |
| Dispatchable STN by dominated-edge filtering (Muscettola, Morris and Tsamardinos 1998, as restated by Morris 2014; minimal except for edge copies on simultaneous points, if that restatement is complete) | done |
| Extended dispatchable STNU: Morris 2014 dispatchable variant, reduced by `minDispESTNU` (without diamonds) | done |
| STNU dynamic controllability: labelled closure with extended dispatchable network (reference) | done |
| STNU dynamic controllability: Morris 2014 `O(n^3)` backpropagation (default checker) | done |
| STNU dynamic controllability: RUL⁻ (Cairo, Hunsberger and Rizzi 2018) | done |
| STNU dynamic controllability: RUL2021 (Hunsberger and Posenato 2022) | done |
| STNU dynamic controllability: per-label backpropagation (cross-check) | done |
| Explanations of non-controllability (semi-reducible negative cycles) | done |
| Onboard dispatcher, `no_std` without `alloc`, fixed capacity, loaded in place from a `const EMPTY` | done |
| `u16`/`u32`/`usize` index types for the dispatcher's internal handles | done |
| Compact checksummed encoding (CRC-32) of dispatchable networks, no-`alloc` decoder | done |
| `cargo fuzz` targets: decoder, dispatcher calls, DC checker agreement, STN algorithm agreement | done |
| Kani harnesses for the dispatcher and the decoder | done: all verify at small bounds (design section 8) |
| Algorithms compared with their papers; the papers' examples as tests (design section 12) | done |
| Differential tests against CSTNU-Tool | planned |

Reaction-time semantics: an observation at tick `t` may trigger executions at the same tick.
Next-tick reaction is supported by transforming the network (`Stnu::with_reaction`).
Every algorithm takes an explicit compute and memory budget.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
