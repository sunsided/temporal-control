---
applyTo: "**/*.md,docs/**"
---

# Documentation

- README files are rendered on crates.io and docs.rs. Every link to a repo file and every
  image uses an absolute URL (`https://github.com/...`, `https://raw.githubusercontent.com/...`).
  Only in-page anchors may be relative.
- Mermaid diagrams (```` ```mermaid ````) must be valid (`task lint:mermaid` renders them)
  and must match the code. Flag diagrams that the change makes stale. Keep each diagram under
  about 15 nodes.
- Code blocks are tagged with their language. The Markdown files hold no Rust code today;
  a change that adds Rust examples to `README.md` must also add a private
  `#[cfg(doctest)]` module in `src/lib.rs` that includes it, so the examples compile. Rust
  examples belong in doc comments, where they run as doctests.
- `docs/design.md` is the specification. A change to an algorithm, a bound or a test
  strategy updates the matching section (status tables, Kani results) in the same PR.
- Keep badges, the License section and the MSRV statement consistent with `Cargo.toml`.
