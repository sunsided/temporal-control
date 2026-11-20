//! # Temporal control
//!
//! Simple temporal networks with uncertainty: dynamic controllability checking and dispatchable
//! execution for `no_std` targets.
//!
//! ## Scope
//!
//! - Simple Temporal Networks ([`Stn`]): consistency checking with a negative-cycle explanation,
//!   minimal networks, incremental consistency ([`IncrementalStn`]) and filtered dispatchable
//!   networks ([`Stn::dispatchable`]).
//! - STNs with Uncertainty ([`Stnu`]): dynamic controllability by two independent checkers
//!   ([`Stnu::dispatchable`], which also yields an extended dispatchable network with waits,
//!   and [`Stnu::check_controllability`]).
//! - An onboard [`Dispatcher`] with fixed capacity that needs no `alloc`.
//! - Planned: explanations of non-controllability, a compact checksummed encoding of
//!   dispatchable networks.
//!
//! Everything except [`Dispatcher`] and its input types needs the default `alloc` feature.
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
//! Pre-release (0.0.0). Milestone 1 (STN) and most of milestones 2 and 3 (STNU, dispatchable
//! form, dispatcher) are implemented; the API will change.
//!
//! Repository: <https://github.com/sunsided/temporal-control>

#![no_std]
#![forbid(unsafe_code)]

#[cfg(feature = "alloc")]
extern crate alloc;

mod budget;
pub mod dispatch;
mod error;
#[cfg(feature = "alloc")]
pub mod stn;
#[cfg(feature = "alloc")]
pub mod stnu;
mod ticks;
mod time_point;

pub use budget::Budget;
#[cfg(feature = "alloc")]
pub use dispatch::DispatchableNetwork;
pub use dispatch::{DispatchEdge, DispatchError, Dispatcher, PointKind, Wait};
pub use error::Error;
#[cfg(feature = "alloc")]
pub use stn::{
    Bound, Constraint, ConstraintId, CycleEdge, IncrementalStn, MinimalNetwork, NegativeCycle,
    Schedule, Stn, Verdict,
};
#[cfg(feature = "alloc")]
pub use stnu::{ContingentId, ContingentLink, Controllability, Stnu};
pub use ticks::Ticks;
pub use time_point::TimePoint;
