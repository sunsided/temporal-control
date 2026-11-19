//! # Temporal control
//!
//! Simple temporal networks with uncertainty: dynamic controllability checking and dispatchable execution for `no_std` targets.
//!
//! ## Planned scope
//!
//! - Simple Temporal Networks (STN)
//! - STNs with Uncertainty (STNU) and dynamic controllability
//! - Dispatchable network generation
//! - Small, bounded-memory onboard dispatcher
//! - Explanations of inconsistency and non-controllability
//!
//! ## Status
//!
//! Pre-release. Version 0.0.0 reserves the name; the API is not started yet and will change.
//!
//! Repository: <https://github.com/sunsided/temporal-control>

#![no_std]

/// A point or duration in time, in integer ticks.
///
/// Integer ticks keep arithmetic deterministic across targets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Ticks(pub i64);
