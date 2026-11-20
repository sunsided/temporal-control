# temporal-control

Simple temporal networks with uncertainty: dynamic controllability checking and dispatchable execution for no_std targets.

## Scope and status

Pre-release (0.0.0) on `draft/initial`; the API will change. See [docs/design.md](docs/design.md).

| Feature | Status |
| --- | --- |
| STN consistency (Bellman-Ford) with negative-cycle explanation | done |
| STN minimal network (Floyd-Warshall) | done |
| Incremental STN consistency, `O(n^2)` per constraint | done |
| Dispatchable STN by dominated-edge filtering (minimal except for edge copies on simultaneous points) | done |
| STNU dynamic controllability: labelled closure with extended dispatchable network | done |
| STNU dynamic controllability: Morris 2014 `O(n^3)` backpropagation (default checker) | done |
| STNU dynamic controllability: per-label backpropagation (cross-check) | done |
| Explanations of non-controllability (semi-reducible negative cycles) | done |
| Onboard dispatcher, `no_std` without `alloc`, fixed capacity | done |
| Compact checksummed encoding of dispatchable networks | planned |
| Differential tests against CSTNU-Tool, fuzzing, Kani harnesses | planned |

Reaction-time semantics: an observation at tick `t` may trigger executions at the same tick.
Every algorithm takes an explicit compute and memory budget.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
