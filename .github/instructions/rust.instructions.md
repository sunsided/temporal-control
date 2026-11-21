---
applyTo: "**/*.rs"
---

# Rust code

- Every module (`//!`), type, field, variant, function, constant and macro has a doc
  comment, private and `pub(crate)` items included. Docs state intent, invariants, units and
  error/panic conditions (`# Errors`, `# Panics`, `# Safety`). They do not restate the name.
- No `mod.rs`: modules with children are `foo.rs` plus `foo/`. Integration tests live under
  `tests/it/`.
- No separator comments (`// -----`, `// =====`). If a file needs sections, split it into
  submodules.
- Leave a blank line after the closing `}` of a block statement when another statement
  follows, and between logical phases of a function. No lumped code.
- One level of abstraction per function. Long loops (simulations, game ticks, solvers) are
  short drivers that call documented step functions. If a block needs a "what" comment,
  extract it into a function named after that comment.
- `clippy::pedantic` clean. Suppression only with a scoped
  `#[expect(lint, reason = "...")]`. Never `#[allow]`, and never crate-level suppression.
- No `unsafe` unless the crate opts out of `#![forbid(unsafe_code)]` with a documented
  reason. Each unsafe block has one operation and a `// SAFETY:` comment.
- The crate is `#![no_std]` with no runtime dependencies. Ground-side items (`stn`, `stnu`,
  `DispatchableNetwork`) sit behind the `alloc` feature. The `Dispatcher`, the decoder and
  their input types must build without `alloc` (the CI `no-std` job). Flag any new
  `[dependencies]` entry, any `std` use, and any feature-gated item without its `#[cfg]` on
  the re-export in `lib.rs`.
- All time arithmetic is checked: overflow becomes `Error::Overflow`, never a wrap or a
  panic. Every algorithm takes a `Budget` and reports exhaustion as an error, never as a
  verdict. Flag unchecked `+`, `-` or `as` casts on `Ticks` values and loops without metering.
- The onboard dispatcher has fixed capacity and validates every input; a rejected call or
  load leaves it unchanged. Changes under `src/dispatch/` must keep the Kani harnesses in
  `src/dispatch/proofs.rs` compiling and should name which harness was re-run.
- The DC checkers are cross-checked against each other, the closure and the brute-force
  oracle (`tests/it/`), and by the `dc_checkers` and `stn` fuzz targets. A new algorithm
  joins those cross-checks in the same PR.
- Public API changes go with a matching update to the README "Scope and status" table and
  `docs/design.md`. Sibling planning crates build against this one by path, so name every
  public API change in the PR.
- Mermaid diagrams in doc comments must match the code they describe. Update them when the
  structure changes.
