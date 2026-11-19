//! # Temporal control
//!
//! Simple temporal networks with uncertainty: dynamic controllability checking and dispatchable
//! execution for `no_std` targets.
//!
//! ## Scope
//!
//! - Simple Temporal Networks (STN): construction, consistency checking with a negative-cycle
//!   explanation, minimal networks. Available now behind the default `alloc` feature.
//! - Planned: STNs with Uncertainty (STNU) and dynamic controllability, dispatchable network
//!   generation, a small bounded-memory onboard dispatcher, explanations of non-controllability.
//!
//! All time values are integer [`Ticks`]. All arithmetic is checked, and every algorithm runs
//! under an explicit [`Budget`]. Exhausting a budget yields [`Error::BudgetExhausted`], never a
//! verdict. See `docs/design.md` in the repository for the formal semantics.
//!
//! ## Example
//!
//! ```
//! # #[cfg(feature = "alloc")] {
//! use temporal_control::{Budget, Stn, Ticks, Verdict};
//!
//! let mut stn = Stn::new();
//! let warmup = stn.add_time_point().expect("fits usize");
//! let analysis = stn.add_time_point().expect("fits usize");
//! // 5 <= t_analysis - t_warmup <= 15
//! stn.add_constraint(warmup, analysis, Some(Ticks(5)), Some(Ticks(15)))
//!     .expect("time points exist");
//!
//! let verdict = stn.check_consistency(&Budget::UNLIMITED).expect("within budget");
//! assert!(matches!(verdict, Verdict::Consistent(_)));
//! # }
//! ```
//!
//! ## Status
//!
//! Pre-release (0.0.0). Milestone 1 (STN) is implemented; the API will change.
//!
//! Repository: <https://github.com/sunsided/temporal-control>

#![no_std]
#![forbid(unsafe_code)]

#[cfg(feature = "alloc")]
extern crate alloc;

mod budget;
mod error;
#[cfg(feature = "alloc")]
pub mod stn;
mod ticks;

pub use budget::Budget;
pub use error::Error;
#[cfg(feature = "alloc")]
pub use stn::{
    Bound, Constraint, ConstraintId, CycleEdge, MinimalNetwork, NegativeCycle, Schedule, Stn,
    TimePoint, Verdict,
};
pub use ticks::Ticks;
