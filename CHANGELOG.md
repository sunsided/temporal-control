# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.1](https://github.com/sunsided/temporal-control/compare/v0.1.0...v0.1.1) - 2026-11-22

### Changed

- Link the crate to its page on the planning and search site

## [0.1.0](https://github.com/sunsided/temporal-control/releases/tag/v0.1.0) - 2026-11-21

### Added

- Add the full lint table, clippy.toml and rustfmt.toml
- Add CONTRIBUTING.md
- Add README badges, usage sections and absolute links
- Add an STN fuzz target
- Add REUSE licensing metadata
- Add cargo-deny, Dependabot and typos configuration
- Add Codecov configuration
- Add Taskfile and CI workflows
- Add P³C partial path consistency for sparse STNs
- Add Kani harnesses for the dispatcher and the decoder
- Add standalone cargo-fuzz targets
- Add a compact checksummed encoding of dispatchable networks
- Support next-tick reaction by transforming the network
- Add the RUL2021 DC checker (Hunsberger and Posenato, AAAI 2022)
- Add the RUL- DC checker (Cairo, Hunsberger and Rizzi 2018)
- Add Morris 2014 O(n^3) DC check with semi-reducible cycle explanations
- Add dispatch and DC benchmarks, update docs for STNU and dispatch
- Add Morris backpropagation as a second DC checker
- Add STNUs with dynamic controllability and extended dispatch
- Add minimal dispatchable STNs and a no-alloc dispatcher
- Add incremental STN consistency
- Add design document
- Add property tests for STN algorithms
- Add STN consistency checking and minimal networks

### Changed

- Describe the semver baseline as the latest crates.io release
- Set the crate version to 0.1.0 for the first release
- Describe the released scope and audience in the README
- Stop tracking Cargo.lock
- Render mermaid diagrams on docs.rs
- Test Ticks arithmetic against an i128 oracle
- Write the bench mains by hand
- Shrink the Kani harnesses until each verifies under 12 GB
- Test links with lo == hi against the oracle, tighten remaining claims
- Validate the algorithms against the papers, fix citations and claims
- Test the examples of the DC and STN papers with their stated answers
- Fuzz every DC and dispatchable-network algorithm, fix an ambiguous doc link
- Test that loading runs on a small stack, tighten CRC claims
- Document in-place loading, index types, encoding, fuzzing and Kani
- Benchmark loading an encoded network in place
- Make the dispatcher's internal index type a parameter
- Load dispatchers in place from a const EMPTY instance
- Keep non-DC explanations in compact form
- Benchmark the three ESTNU algorithms on the chain
- Build the STNU dispatchable network with Morris 2014, then reduce it
- Use Morris's normal form in the O(n^3) checker, document its costs
- Update the design doc for the new DC checkers and explanations
- Budget closure snapshots, test the dispatcher boundary, fix claims
- Tidy the Stnu doctest
- Check that budgets never change the backprop verdict
- Check time point count, tighten design claims
- Reserve the crate name with a 0.0.0 placeholder

### Fixed

- Fix dispatch deadlock through zero-weight dominated edges
