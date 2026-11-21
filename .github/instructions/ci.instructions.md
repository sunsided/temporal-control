---
applyTo: ".github/workflows/**,Taskfile.dist.yaml,**/Taskfile.dist.yaml,.pre-commit-config.yaml"
---

# CI, Taskfile and hooks stay in sync

- The `ci` task in `Taskfile.dist.yaml` mirrors the jobs in `.github/workflows/ci.yml`. A
  change that adds, removes or renames a CI job, or changes its flags, must make the same
  change to `task ci` and the task it calls, and the other way round. Flag any drift in review.
- Local hooks in `.pre-commit-config.yaml` call Taskfile tasks. They must not duplicate
  commands.
- Every `uses:` is pinned to a full 40-character commit SHA, with a `# vX.Y.Z` comment. Reject
  tag or branch refs.
- `dtolnay/rust-toolchain` is pinned to the commit of its `stable` or `nightly` branch
  (`# stable`, `# nightly`) and always gets an explicit `toolchain:` input that matches the
  comment. Those branches are not tags, so check Dependabot bumps of this action: a `nightly`
  use must not be re-pinned to the `stable` commit.
- The workflow sets `permissions: {}` at the top, and each job sets the least privilege it
  needs. Every `actions/checkout` sets `persist-credentials: false`.
- No `pull_request_target`. No `${{ github.event.* }}` or other untrusted expressions
  directly in `run:`. Pass them through `env:`.
- Do not rename `publish-crates-io.yml` or its `crates-io` environment. crates.io trusted publishing is
  bound to both. Deploy jobs run in their own tag-restricted environment, and `needs:` a
  verification job.
- CI jobs: `branch`, `fmt`, `clippy`, `style`, `doc`, `test` (Linux only), `coverage`,
  `deny`, `msrv`, `semver`, `no-std`, `fuzz`, plus `actionlint` and `zizmor` in
  `actions-lint.yml`.
- `no-std` mirrors `task build:no-std`: the library for `thumbv7em-none-eabihf`, with
  `--no-default-features` and with the default `alloc` feature. Do not drop either build.
- Kani stays out of CI: single harnesses need up to 9 GB. `task kani HARNESS=<name>` runs one
  under a 12 GB cap (`docs/design.md`, section 8). Reject a CI job or task that runs
  `cargo kani` without `--harness`.
- `msrv` checks the library only (`cargo hack check --rust-version --all-features`, no
  `--all-targets`): dev-dependencies may need a newer Rust (criterion 0.8 needs 1.86). Do not
  add `--all-targets` back.
- `fuzz/` is a separate nightly workspace. Its tasks live in `fuzz/Taskfile.dist.yaml`,
  included as `fuzz:`; CI's `fuzz` job mirrors `task fuzz:build` and `task fuzz:lint`.
- `Cargo.lock` is tracked (root and `fuzz/`), and Dependabot updates both. `ci.yml` steps do
  not use `--locked`; `publish-crates-io.yml` does, so a release builds exactly the locked
  dependencies.
- `scripts/check-branch.sh` is the single definition of allowed branch names. It is used by
  the Taskfile, the hook and the CI `branch` job.
- New jobs that should gate merges must also be added to branch protection. Mention that in
  the PR.
