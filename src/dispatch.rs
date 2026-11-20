//! Onboard dispatcher: executes a dispatchable network online, without allocation.
//!
//! A *dispatchable network* is a set of [`DispatchEdge`]s (`t_to - t_from <= weight`) and
//! [`Wait`]s over executable and contingent time points. The ground side derives it from an
//! [`Stn`](crate::Stn) or a dynamically controllable [`Stnu`](crate::Stnu); the onboard side only
//! loads and executes it.
//!
//! # Execution semantics
//!
//! The dispatcher keeps, per time point, a window `[lb, ub]` (initially unbounded) and the
//! number of precedence edges `x -> y` (defined below) whose head `y` has not happened yet. When a time point
//! happens at `t` (executed by the agent or observed for a contingent point), the dispatcher
//! propagates only along that point's own edges:
//!
//! - edge `x -> y` with weight `w` and `x` happened: `ub[y] = min(ub[y], t + w)`;
//! - edge `y -> x` with weight `w` and `x` happened: `lb[y] = max(lb[y], t - w)`.
//!
//! An edge `x -> y` with weight `w < 0` is a *precedence* edge: `y` must happen strictly
//! before `x`. A consistent network has no cycle of precedence edges. Zero-weight edges are
//! not precedence edges; `x` may happen first and then caps `y`'s window at its own time.
//!
//! An executable point `x` may execute at `now` iff it has not happened, every precedence edge
//! `x -> y` has `y` happened, `lb[x] <= now <= ub[x]`, and every wait `(x, c, d)` is satisfied:
//! `c` has been observed, or its activation point happened at `a` and `now >= a + d`.
//!
//! **Reaction time.** Within one tick, observations are processed first, so a point may execute
//! in the same tick in which a contingent point it waits for was observed (instantaneous
//! reaction at tick granularity). [`Dispatcher::step`] follows that order.
//!
//! ```mermaid
//! stateDiagram-v2
//!     [*] --> Waiting
//!     Waiting --> Enabled: all negative-edge heads happened
//!     Enabled --> Executable: lb <= now, waits satisfied
//!     Executable --> Happened: execute(now)
//!     Executable --> Missed: now > ub
//!     Happened --> [*]
//! ```
//!
//! # Memory
//!
//! [`Dispatcher`] has fixed capacities: `N` time points and `E` adjacency entries. Every edge
//! takes two entries (one per endpoint) and every wait one. Work per event is `O(degree)`;
//! [`Dispatcher::step`] is `O(N * degree)` per executed point. Internal handles are stored in
//! an [`Index`] type (`usize` by default, or `u16`/`u32`).

pub mod encoding;
mod index;
#[cfg(feature = "alloc")]
mod network;
mod source;

use core::fmt;

use crate::{Ticks, TimePoint};
use source::{Slices, Source};

pub use encoding::{DecodeError, EncodeError};
pub use index::Index;
#[cfg(feature = "alloc")]
pub use network::DispatchableNetwork;

/// What kind of time point a dispatcher node is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointKind {
    /// The agent decides when this point happens.
    Executable,
    /// Nature decides: once `activation` happened at `a`, this point happens in `[a + lo, a + hi]`.
    Contingent {
        /// The time point that starts the contingent duration.
        activation: TimePoint,
        /// Shortest duration. Positive.
        lo: Ticks,
        /// Longest duration. At least `lo`.
        hi: Ticks,
    },
}

/// A dispatchable edge `t_to - t_from <= weight`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DispatchEdge {
    /// Tail of the distance-graph edge.
    pub from: TimePoint,
    /// Head of the distance-graph edge.
    pub to: TimePoint,
    /// Maximum of `t_to - t_from`.
    pub weight: Ticks,
}

/// A wait: `waiter` must not execute before `activation(contingent) + delay`, unless
/// `contingent` has been observed first.
///
/// Waits are the upper-case edges of an extended STNU (Morris, Muscettola and Vidal 2001).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Wait {
    /// The executable point that waits.
    pub waiter: TimePoint,
    /// The contingent point being waited for.
    pub contingent: TimePoint,
    /// Wait length after the contingent point's activation.
    pub delay: Ticks,
}

/// Reasons a dispatcher call failed or detected a problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum DispatchError {
    /// The network needs more than `N` points or `E` entries.
    CapacityExceeded,
    /// The network has more points or entries than the dispatcher's [`Index`] type can
    /// address.
    IndexTypeTooSmall,
    /// The network is malformed: a bad index, a contingent link with `lo <= 0` or `lo > hi`, a
    /// cyclic activation chain, a negative self-loop, or a wait on a non-contingent point.
    InvalidNetwork,
    /// The handle is not part of this dispatcher, or has the wrong kind for the call.
    UnknownTimePoint,
    /// `now` is earlier than a time passed to an earlier call.
    TimeWentBackwards,
    /// [`Dispatcher::execute`] was asked to execute a point that may not execute now. Nothing
    /// changed.
    NotExecutable(TimePoint),
    /// The point already happened.
    AlreadyHappened(TimePoint),
    /// An executable point's window closed before it executed.
    DeadlineMissed(TimePoint),
    /// A contingent point was observed outside, or failed to occur within, its declared bounds.
    /// The network's assumptions do not hold; its guarantees are void.
    ContingentOutOfBounds(TimePoint),
    /// A constraint of the network is violated, or the window of the point became empty.
    ConstraintViolated(TimePoint),
    /// The output buffer passed to [`Dispatcher::step`] is full.
    OutputFull,
    /// Tick arithmetic overflowed.
    Overflow,
}

impl fmt::Display for DispatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CapacityExceeded => f.write_str("dispatcher capacity exceeded"),
            Self::IndexTypeTooSmall => f.write_str("network too large for the index type"),
            Self::InvalidNetwork => f.write_str("malformed dispatchable network"),
            Self::UnknownTimePoint => f.write_str("unknown time point or wrong kind"),
            Self::TimeWentBackwards => f.write_str("time went backwards"),
            Self::NotExecutable(tp) => write!(f, "time point {} may not execute now", tp.0),
            Self::AlreadyHappened(tp) => write!(f, "time point {} already happened", tp.0),
            Self::DeadlineMissed(tp) => write!(f, "deadline of time point {} missed", tp.0),
            Self::ContingentOutOfBounds(tp) => {
                write!(f, "contingent time point {} outside its bounds", tp.0)
            }
            Self::ConstraintViolated(tp) => write!(f, "constraint on time point {} violated", tp.0),
            Self::OutputFull => f.write_str("output buffer full"),
            Self::Overflow => f.write_str("tick arithmetic overflowed"),
        }
    }
}

impl core::error::Error for DispatchError {}

/// Kind of a loaded point, with the activation stored as an [`Index`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind<I> {
    /// See [`PointKind::Executable`].
    Executable,
    /// See [`PointKind::Contingent`].
    Contingent {
        /// The time point that starts the contingent duration.
        activation: I,
        /// Shortest duration.
        lo: Ticks,
        /// Longest duration.
        hi: Ticks,
    },
}

impl<I: Index> Kind<I> {
    /// Narrows a validated [`PointKind`].
    ///
    /// # Errors
    ///
    /// [`DispatchError::IndexTypeTooSmall`] if the activation does not fit `I`.
    fn narrow(kind: PointKind) -> Result<Self, DispatchError> {
        Ok(match kind {
            PointKind::Executable => Self::Executable,
            PointKind::Contingent { activation, lo, hi } => Self::Contingent {
                activation: index(activation.0)?,
                lo,
                hi,
            },
        })
    }

    /// Widens back to the public [`PointKind`].
    fn widen(self) -> PointKind {
        match self {
            Self::Executable => PointKind::Executable,
            Self::Contingent { activation, lo, hi } => PointKind::Contingent {
                activation: TimePoint(activation.to_usize()),
                lo,
                hi,
            },
        }
    }

    /// Whether this is an executable point.
    const fn is_executable(self) -> bool {
        matches!(self, Self::Executable)
    }
}

/// One adjacency entry, owned by one time point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Entry<I> {
    /// Unused slot.
    Empty,
    /// When the owner happens at `t`: `ub[target] <= t + weight`.
    Upper {
        /// The point whose upper bound tightens.
        target: I,
        /// Edge weight.
        weight: Ticks,
    },
    /// When the owner happens at `t`: `lb[target] >= t - weight`.
    Lower {
        /// The point whose lower bound tightens.
        target: I,
        /// Weight of the edge `target -> owner`.
        weight: Ticks,
    },
    /// The owner waits for `contingent` until `activation + delay`.
    Wait {
        /// The contingent point waited for.
        contingent: I,
        /// Wait length after activation.
        delay: Ticks,
    },
}

/// Fixed-capacity online dispatcher for up to `N` time points and `E` adjacency entries,
/// storing internal handles as `I` (see [`Index`]).
///
/// # Examples
///
/// ```
/// use temporal_control::{DispatchEdge, Dispatcher, PointKind, Ticks, TimePoint};
///
/// // 5 <= t_1 - t_0 <= 15
/// let points = [PointKind::Executable; 2];
/// let edges = [
///     DispatchEdge { from: TimePoint(0), to: TimePoint(1), weight: Ticks(15) },
///     DispatchEdge { from: TimePoint(1), to: TimePoint(0), weight: Ticks(-5) },
/// ];
/// let mut d = Dispatcher::<2, 4>::new(&points, &edges, &[]).expect("fits");
/// let mut out = [TimePoint(0); 2];
///
/// assert_eq!(d.step(Ticks(0), &[], &mut out), Ok(1)); // t_0 = 0
/// assert_eq!(d.step(Ticks(4), &[], &mut out), Ok(0)); // too early for t_1
/// assert_eq!(d.step(Ticks(5), &[], &mut out), Ok(1)); // t_1 = 5
/// assert!(d.is_complete());
/// ```
#[derive(Debug, Clone)]
pub struct Dispatcher<const N: usize, const E: usize, I: Index = usize> {
    /// Number of time points in use.
    n: usize,
    /// Kind of each point.
    kinds: [Kind<I>; N],
    /// Lower window bound per point; `None` is `-infinity`.
    lb: [Option<Ticks>; N],
    /// Upper window bound per point; `None` is `+infinity`.
    ub: [Option<Ticks>; N],
    /// Time each point happened, `None` until it happens.
    time: [Option<Ticks>; N],
    /// Per point: outgoing precedence edges whose head has not happened yet.
    pending: [I; N],
    /// Start of each point's entries in `entries`; the end is the next point's start.
    start: [I; N],
    /// Number of entries in use.
    m: usize,
    /// Adjacency entries grouped by owner.
    entries: [Entry<I>; E],
    /// Latest time passed to any call; `None` before the first call.
    now: Option<Ticks>,
}

impl<const N: usize, const E: usize, I: Index> Dispatcher<N, E, I> {
    /// A dispatcher with no network loaded, usable in `const` and `static` initialisers.
    ///
    /// Flight software typically places the dispatcher in statically allocated memory once
    /// and then calls [`Self::load`] for every new plan, so the (possibly large) structure is
    /// never built on the stack and returned by value. With no network loaded the dispatcher
    /// has no points, [`Self::is_complete`] is true and every point handle is unknown.
    ///
    /// # Examples
    ///
    /// ```
    /// use temporal_control::{DispatchEdge, Dispatcher, PointKind, Ticks, TimePoint};
    ///
    /// // Built at compile time; a flight build would put this behind its own cell type.
    /// static IDLE: Dispatcher<8, 32> = Dispatcher::EMPTY;
    /// assert!(IDLE.is_complete());
    ///
    /// let mut d = Dispatcher::<8, 32>::EMPTY;
    /// let edges = [DispatchEdge { from: TimePoint(0), to: TimePoint(1), weight: Ticks(10) }];
    /// d.load(&[PointKind::Executable; 2], &edges, &[]).expect("fits");
    /// assert!(!d.is_complete());
    /// ```
    pub const EMPTY: Self = Self {
        n: 0,
        kinds: [Kind::Executable; N],
        lb: [None; N],
        ub: [None; N],
        time: [None; N],
        pending: [I::ZERO; N],
        start: [I::ZERO; N],
        m: 0,
        entries: [Entry::Empty; E],
        now: None,
    };

    /// Validates a dispatchable network and loads it into a new dispatcher.
    ///
    /// Shorthand for [`Self::EMPTY`] followed by [`Self::load`]. The dispatcher is returned by
    /// value; for large capacities on a small stack prefer [`Self::load`] on a dispatcher in
    /// static memory.
    ///
    /// # Errors
    ///
    /// As for [`Self::load`].
    pub fn new(
        points: &[PointKind],
        edges: &[DispatchEdge],
        waits: &[Wait],
    ) -> Result<Self, DispatchError> {
        let mut d = Self::EMPTY;
        d.load(points, edges, waits)?;
        Ok(d)
    }

    /// Validates a dispatchable network and loads it in place, replacing any earlier network
    /// and execution state.
    ///
    /// This is the onboard trust boundary: every index, bound and capacity is checked, and
    /// nothing is assumed about how the network was produced. Validation completes before the
    /// first write, so a rejected network leaves the dispatcher exactly as it was (still
    /// executing the previous plan, if any). Loading needs no stack memory proportional to
    /// `N` or `E`.
    ///
    /// # Errors
    ///
    /// - [`DispatchError::CapacityExceeded`] if `points.len() > N` or
    ///   `2 * edges.len() + waits.len() > E`.
    /// - [`DispatchError::IndexTypeTooSmall`] if the point or entry count exceeds the maximum
    ///   of `I` (see [`Index`]).
    /// - [`DispatchError::InvalidNetwork`] for any malformed input (see its docs).
    pub fn load(
        &mut self,
        points: &[PointKind],
        edges: &[DispatchEdge],
        waits: &[Wait],
    ) -> Result<(), DispatchError> {
        self.load_from(&Slices {
            points,
            edges,
            waits,
        })
    }

    /// Validates an encoded dispatchable network and loads it in place, without copying it
    /// anywhere first.
    ///
    /// `bytes` must be exactly one encoding as written by [`encoding::encode_into`] (see
    /// [`encoding`] for the format). Header, length and CRC-32 are checked first, then every
    /// record, then the network as in [`Self::load`]. As there, a rejected encoding leaves the
    /// dispatcher exactly as it was. Never panics on any input.
    ///
    /// # Examples
    ///
    /// ```
    /// use temporal_control::dispatch::encoding;
    /// use temporal_control::{DecodeError, DispatchEdge, Dispatcher, PointKind, Ticks, TimePoint};
    ///
    /// let points = [PointKind::Executable; 2];
    /// let edges = [DispatchEdge { from: TimePoint(0), to: TimePoint(1), weight: Ticks(10) }];
    /// let mut buf = [0u8; 128];
    /// let len = encoding::encode_into(&points, &edges, &[], &mut buf).expect("fits");
    ///
    /// let mut d = Dispatcher::<4, 8>::EMPTY;
    /// d.load_encoded(&buf[..len]).expect("valid");
    /// assert_eq!(d.window(TimePoint(1)), Some((None, None)));
    ///
    /// buf[30] ^= 0x10;
    /// assert_eq!(d.load_encoded(&buf[..len]), Err(DecodeError::ChecksumMismatch));
    /// ```
    ///
    /// # Errors
    ///
    /// Any [`DecodeError`]; [`DecodeError::Network`] wraps the errors of [`Self::load`].
    pub fn load_encoded(&mut self, bytes: &[u8]) -> Result<(), DecodeError> {
        let encoded = encoding::Encoded::parse(bytes)?;
        self.load_from(&encoded).map_err(DecodeError::Network)
    }

    /// Validates `net` and loads it in place; see [`Self::load`].
    ///
    /// # Errors
    ///
    /// As for [`Self::load`].
    fn load_from(&mut self, net: &impl Source) -> Result<(), DispatchError> {
        source::validate::<I>(net, N, E)?;

        self.clear();
        let filled = self.fill(net);

        if filled.is_err() {
            self.clear();
        }

        filled
    }

    /// Records that contingent point `tp` was observed at `now`.
    ///
    /// The observation is recorded and propagated even when it violates an assumption, so that
    /// monitoring continues; the violation is then reported as an error.
    ///
    /// # Errors
    ///
    /// - [`DispatchError::UnknownTimePoint`] if `tp` is not a contingent point of this network.
    /// - [`DispatchError::AlreadyHappened`] if `tp` was already observed.
    /// - [`DispatchError::TimeWentBackwards`] if `now` precedes an earlier call.
    /// - [`DispatchError::ContingentOutOfBounds`] if the activation has not happened or `now`
    ///   is outside `[a + lo, a + hi]`.
    /// - [`DispatchError::ConstraintViolated`] if a constraint with an already happened point is
    ///   violated or a window became empty.
    /// - [`DispatchError::Overflow`] on tick overflow.
    pub fn observe(&mut self, tp: TimePoint, now: Ticks) -> Result<(), DispatchError> {
        let PointKind::Contingent {
            activation, lo, hi, ..
        } = self.kind(tp)?
        else {
            return Err(DispatchError::UnknownTimePoint);
        };

        self.ensure_unhappened(tp)?;
        self.advance_clock(now)?;

        let in_bounds = self.time[activation.0].is_some_and(|a| {
            a.checked_add(lo).is_some_and(|l| l <= now)
                && a.checked_add(hi).is_none_or(|h| now <= h)
        });
        let in_window = self.in_window(tp.0, now);
        self.happen(tp.0, now)?;

        if !in_bounds {
            return Err(DispatchError::ContingentOutOfBounds(tp));
        }

        if !in_window {
            return Err(DispatchError::ConstraintViolated(tp));
        }

        Ok(())
    }

    /// Whether executable point `tp` may execute at `now`.
    #[must_use]
    pub fn can_execute(&self, tp: TimePoint, now: Ticks) -> bool {
        tp.0 < self.n
            && self.kinds[tp.0].is_executable()
            && self.time[tp.0].is_none()
            && self.now.is_none_or(|t| t <= now)
            && self.pending[tp.0] == I::ZERO
            && self.in_window(tp.0, now)
            && self.waits_satisfied(tp.0, now)
    }

    /// Executes `tp` at `now` and propagates to its neighbors.
    ///
    /// # Errors
    ///
    /// - [`DispatchError::NotExecutable`] if [`Self::can_execute`] is false; nothing changes.
    /// - [`DispatchError::ConstraintViolated`] if propagation empties a window. The point is
    ///   executed regardless; this means the network was not dispatchable.
    /// - [`DispatchError::Overflow`] on tick overflow.
    pub fn execute(&mut self, tp: TimePoint, now: Ticks) -> Result<(), DispatchError> {
        if !self.can_execute(tp, now) {
            return Err(DispatchError::NotExecutable(tp));
        }

        self.now = Some(now);
        self.happen(tp.0, now)
    }

    /// Advances the clock to `now` and reports deadlines that have passed.
    ///
    /// # Errors
    ///
    /// - [`DispatchError::TimeWentBackwards`] if `now` precedes an earlier call.
    /// - [`DispatchError::DeadlineMissed`] for the first executable point with `ub < now`.
    /// - [`DispatchError::ContingentOutOfBounds`] for the first contingent point that has not
    ///   been observed although `now > a + hi`.
    pub fn check_deadlines(&mut self, now: Ticks) -> Result<(), DispatchError> {
        self.advance_clock(now)?;

        for p in 0..self.n {
            if self.time[p].is_some() {
                continue;
            }

            let deadline = match self.kinds[p] {
                Kind::Executable => self.ub[p],
                Kind::Contingent { activation, hi, .. } => {
                    self.time[activation.to_usize()].and_then(|a| a.checked_add(hi))
                }
            };

            if deadline.is_some_and(|d| d < now) {
                return Err(match self.kinds[p] {
                    Kind::Executable => DispatchError::DeadlineMissed(TimePoint(p)),
                    Kind::Contingent { .. } => DispatchError::ContingentOutOfBounds(TimePoint(p)),
                });
            }
        }

        Ok(())
    }

    /// Runs one tick with the as-soon-as-possible policy.
    ///
    /// Order: record every point in `observed` at `now`, check deadlines, then execute every
    /// point that may execute at `now`, repeating until none is left. Executed points are
    /// written to `out`; the return value is their number.
    ///
    /// # Errors
    ///
    /// Any error of [`Self::observe`] or [`Self::check_deadlines`], which stops the tick, and
    /// [`DispatchError::OutputFull`] if `out` cannot hold the next executed point (that point
    /// is not executed). A buffer of `N` entries never fills up.
    pub fn step(
        &mut self,
        now: Ticks,
        observed: &[TimePoint],
        out: &mut [TimePoint],
    ) -> Result<usize, DispatchError> {
        for &c in observed {
            self.observe(c, now)?;
        }

        self.check_deadlines(now)?;

        let mut count = 0;

        while let Some(tp) = self.next_executable(now) {
            let slot = out.get_mut(count).ok_or(DispatchError::OutputFull)?;
            *slot = tp;
            count += 1;
            self.execute(tp, now)?;
        }

        Ok(count)
    }

    /// Earliest time at or after the latest call at which some executable point could execute
    /// without new observations, or `None` if none can.
    ///
    /// An executive may sleep until then unless an observation arrives first.
    #[must_use]
    pub fn next_wakeup(&self) -> Option<Ticks> {
        let floor = self.now.unwrap_or(Ticks(i64::MIN));

        (0..self.n)
            .filter(|&p| self.kinds[p].is_executable() && self.time[p].is_none())
            .filter(|&p| self.pending[p] == I::ZERO)
            .filter_map(|p| self.earliest_start(p, floor))
            .min()
    }

    /// Time at which `tp` happened, or `None` if it has not (or is unknown).
    #[must_use]
    pub fn time(&self, tp: TimePoint) -> Option<Ticks> {
        self.time.get(..self.n)?.get(tp.0).copied().flatten()
    }

    /// Current window `(lb, ub)` of `tp`; `None` bounds are infinite. `None` for unknown handles.
    #[must_use]
    pub fn window(&self, tp: TimePoint) -> Option<(Option<Ticks>, Option<Ticks>)> {
        (tp.0 < self.n).then(|| (self.lb[tp.0], self.ub[tp.0]))
    }

    /// Whether every time point has happened.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.time[..self.n].iter().all(Option::is_some)
    }

    /// Looks up the kind of `tp`.
    ///
    /// # Errors
    ///
    /// [`DispatchError::UnknownTimePoint`] if `tp` is out of range.
    fn kind(&self, tp: TimePoint) -> Result<PointKind, DispatchError> {
        self.kinds[..self.n]
            .get(tp.0)
            .map(|k| k.widen())
            .ok_or(DispatchError::UnknownTimePoint)
    }

    /// Fails if `tp` already happened.
    ///
    /// # Errors
    ///
    /// [`DispatchError::AlreadyHappened`].
    fn ensure_unhappened(&self, tp: TimePoint) -> Result<(), DispatchError> {
        if self.time[tp.0].is_some() {
            return Err(DispatchError::AlreadyHappened(tp));
        }

        Ok(())
    }

    /// Moves the clock to `now`.
    ///
    /// # Errors
    ///
    /// [`DispatchError::TimeWentBackwards`] if `now` precedes the clock.
    fn advance_clock(&mut self, now: Ticks) -> Result<(), DispatchError> {
        if self.now.is_some_and(|t| now < t) {
            return Err(DispatchError::TimeWentBackwards);
        }

        self.now = Some(now);
        Ok(())
    }

    /// Whether `now` lies in the window of point `p`.
    fn in_window(&self, p: usize, now: Ticks) -> bool {
        self.lb[p].is_none_or(|l| l <= now) && self.ub[p].is_none_or(|u| now <= u)
    }

    /// Index range in `entries` of the entries owned by point `p`.
    fn range_of(&self, p: usize) -> core::ops::Range<usize> {
        let end = if p + 1 < self.n {
            self.start[p + 1].to_usize()
        } else {
            self.m
        };

        self.start[p].to_usize()..end
    }

    /// Entries owned by point `p`.
    fn entries_of(&self, p: usize) -> &[Entry<I>] {
        &self.entries[self.range_of(p)]
    }

    /// Whether every wait of `p` is satisfied at `now`.
    fn waits_satisfied(&self, p: usize, now: Ticks) -> bool {
        self.entries_of(p).iter().all(|e| match *e {
            Entry::Wait { contingent, delay } => {
                self.wait_released(contingent.to_usize(), delay, now)
            }
            _ => true,
        })
    }

    /// Whether a wait for `contingent` with `delay` is over at `now`.
    fn wait_released(&self, contingent: usize, delay: Ticks, now: Ticks) -> bool {
        self.wait_release_time(contingent, delay)
            .is_some_and(|r| r <= now)
    }

    /// Time at which a wait for `contingent` with `delay` ends: already (`i64::MIN`) if the
    /// contingent point happened, `a + delay` once its activation happened, else `None`.
    fn wait_release_time(&self, contingent: usize, delay: Ticks) -> Option<Ticks> {
        if self.time[contingent].is_some() {
            return Some(Ticks(i64::MIN));
        }

        let Kind::Contingent { activation, .. } = self.kinds[contingent] else {
            return None;
        };

        self.time[activation.to_usize()].and_then(|a| a.checked_add(delay))
    }

    /// Earliest time `>= floor` at which enabled point `p` could execute, if its window allows.
    fn earliest_start(&self, p: usize, floor: Ticks) -> Option<Ticks> {
        let mut t = self.lb[p].map_or(floor, |l| l.max(floor));

        for e in self.entries_of(p) {
            if let Entry::Wait { contingent, delay } = *e {
                t = t.max(self.wait_release_time(contingent.to_usize(), delay)?);
            }
        }

        self.ub[p].is_none_or(|u| t <= u).then_some(t)
    }

    /// First point that may execute at `now`, in index order.
    fn next_executable(&self, now: Ticks) -> Option<TimePoint> {
        (0..self.n)
            .map(TimePoint)
            .find(|&tp| self.can_execute(tp, now))
    }

    /// Marks `p` as happened at `t` and propagates along its entries.
    ///
    /// Propagation always completes; the first problem found is returned afterwards.
    ///
    /// # Errors
    ///
    /// [`DispatchError::ConstraintViolated`] or [`DispatchError::Overflow`].
    fn happen(&mut self, p: usize, t: Ticks) -> Result<(), DispatchError> {
        self.time[p] = Some(t);
        let mut result = Ok(());

        for i in self.range_of(p) {
            let outcome = match self.entries[i] {
                Entry::Upper { target, weight } => self.tighten_upper(target.to_usize(), t, weight),
                Entry::Lower { target, weight } => {
                    self.tighten_lower(p, target.to_usize(), t, weight)
                }
                Entry::Wait { .. } | Entry::Empty => Ok(()),
            };

            if result.is_ok() {
                result = outcome;
            }
        }

        result
    }

    /// Applies `t_target <= t + weight`.
    ///
    /// # Errors
    ///
    /// [`DispatchError::ConstraintViolated`] if `target` happened later than allowed or its
    /// window becomes empty; [`DispatchError::Overflow`] if the bound underflows.
    fn tighten_upper(
        &mut self,
        target: usize,
        t: Ticks,
        weight: Ticks,
    ) -> Result<(), DispatchError> {
        let Some(bound) = t.checked_add(weight) else {
            // Overflow upwards means no bound; downwards means an unmeetable one.
            return if weight.0 > 0 {
                Ok(())
            } else {
                Err(DispatchError::Overflow)
            };
        };

        if let Some(s) = self.time[target] {
            return ok_or_violated(s <= bound, target);
        }

        if self.ub[target].is_none_or(|u| bound < u) {
            self.ub[target] = Some(bound);
        }

        ok_or_violated(self.window_nonempty(target), target)
    }

    /// Applies `t_target >= t - weight`, where `owner` happened at `t`, and releases the
    /// precedence edge `target -> owner` if it is one.
    ///
    /// # Errors
    ///
    /// As for [`Self::tighten_upper`].
    fn tighten_lower(
        &mut self,
        owner: usize,
        target: usize,
        t: Ticks,
        weight: Ticks,
    ) -> Result<(), DispatchError> {
        if is_precedence(target, owner, weight) && self.time[target].is_none() {
            self.pending[target] = self.pending[target].saturating_dec();
        }

        let Some(bound) = weight.checked_neg().and_then(|w| t.checked_add(w)) else {
            return if weight.0 > 0 {
                Ok(())
            } else {
                Err(DispatchError::Overflow)
            };
        };

        if let Some(s) = self.time[target] {
            return ok_or_violated(bound <= s, target);
        }

        if self.lb[target].is_none_or(|l| bound > l) {
            self.lb[target] = Some(bound);
        }

        ok_or_violated(self.window_nonempty(target), target)
    }

    /// Whether `lb[p] <= ub[p]`.
    fn window_nonempty(&self, p: usize) -> bool {
        match (self.lb[p], self.ub[p]) {
            (Some(l), Some(u)) => l <= u,
            _ => true,
        }
    }

    /// Resets every field to the state of [`Self::EMPTY`], in place.
    fn clear(&mut self) {
        self.n = 0;
        self.kinds.fill(Kind::Executable);
        self.lb.fill(None);
        self.ub.fill(None);
        self.time.fill(None);
        self.pending.fill(I::ZERO);
        self.start.fill(I::ZERO);
        self.m = 0;
        self.entries.fill(Entry::Empty);
        self.now = None;
    }

    /// Fills a cleared dispatcher from a validated network.
    ///
    /// # Errors
    ///
    /// [`DispatchError::IndexTypeTooSmall`] if a value does not fit `I`, or
    /// [`DispatchError::InvalidNetwork`] if a record cannot be read. Validation already rules
    /// both out; the errors replace what would otherwise be panic paths, and the caller clears
    /// the half-filled state.
    fn fill(&mut self, net: &impl Source) -> Result<(), DispatchError> {
        for (p, slot) in self.kinds.iter_mut().take(net.point_count()).enumerate() {
            *slot = Kind::narrow(record(net.point(p))?)?;
        }

        self.n = net.point_count();
        self.count_entries(net)?;
        self.fill_entries(net)
    }

    /// Sets `start[p]` to the end of `p`'s entry range, `m` to the entry count, and `pending`
    /// to the number of precedence edges per point.
    ///
    /// # Errors
    ///
    /// As for [`Self::fill`].
    fn count_entries(&mut self, net: &impl Source) -> Result<(), DispatchError> {
        for i in 0..net.edge_count() {
            let e = record(net.edge(i))?;
            increment(&mut self.start[e.from.0])?;
            increment(&mut self.start[e.to.0])?;

            if is_precedence(e.from.0, e.to.0, e.weight) {
                increment(&mut self.pending[e.from.0])?;
            }
        }

        for i in 0..net.wait_count() {
            increment(&mut self.start[record(net.wait(i))?.waiter.0])?;
        }

        let mut end = 0usize;

        for p in 0..self.n {
            end += self.start[p].to_usize();
            self.start[p] = index(end)?;
        }

        self.m = end;
        Ok(())
    }

    /// Writes the owner-grouped adjacency, moving each `start[p]` from the end to the
    /// beginning of `p`'s range.
    ///
    /// Together with [`Self::count_entries`] this is a counting sort that works inside `start`
    /// instead of on temporary arrays. Iterating the input in reverse keeps each owner's
    /// entries in input order (its edge entries in edge order, then its waits), which decides
    /// which violation [`Self::happen`] reports first.
    ///
    /// # Errors
    ///
    /// As for [`Self::fill`].
    fn fill_entries(&mut self, net: &impl Source) -> Result<(), DispatchError> {
        for i in (0..net.wait_count()).rev() {
            let w = record(net.wait(i))?;
            let entry = Entry::Wait {
                contingent: index(w.contingent.0)?,
                delay: w.delay,
            };
            self.push_back(w.waiter.0, entry);
        }

        for i in (0..net.edge_count()).rev() {
            let e = record(net.edge(i))?;
            let (u, v) = (e.from.0, e.to.0);
            let lower = Entry::Lower {
                target: index(u)?,
                weight: e.weight,
            };
            let upper = Entry::Upper {
                target: index(v)?,
                weight: e.weight,
            };
            self.push_back(v, lower);
            self.push_back(u, upper);
        }

        Ok(())
    }

    /// Writes `entry` just before the current `start[owner]` and moves `start[owner]` there.
    fn push_back(&mut self, owner: usize, entry: Entry<I>) {
        let slot = self.start[owner].saturating_dec();
        self.start[owner] = slot;
        self.entries[slot.to_usize()] = entry;
    }
}

/// Unwraps a record read from a validated [`Source`].
///
/// # Errors
///
/// [`DispatchError::InvalidNetwork`] if it could not be read.
fn record<T>(value: Option<T>) -> Result<T, DispatchError> {
    value.ok_or(DispatchError::InvalidNetwork)
}

/// Converts `value` to the index type.
///
/// # Errors
///
/// [`DispatchError::IndexTypeTooSmall`] if it does not fit.
fn index<I: Index>(value: usize) -> Result<I, DispatchError> {
    I::from_usize(value).ok_or(DispatchError::IndexTypeTooSmall)
}

/// Adds one to `counter`.
///
/// # Errors
///
/// [`DispatchError::IndexTypeTooSmall`] if the result does not fit.
fn increment<I: Index>(counter: &mut I) -> Result<(), DispatchError> {
    *counter = index(counter.to_usize().saturating_add(1))?;
    Ok(())
}

/// Whether edge `from -> to` with `weight` makes `from` wait for `to` (see the module docs).
fn is_precedence(from: usize, to: usize, weight: Ticks) -> bool {
    weight < Ticks::ZERO && from != to
}

/// `Ok(())` if `ok`, else [`DispatchError::ConstraintViolated`] for point `p`.
///
/// # Errors
///
/// [`DispatchError::ConstraintViolated`] when `ok` is false.
fn ok_or_violated(ok: bool, p: usize) -> Result<(), DispatchError> {
    if ok {
        Ok(())
    } else {
        Err(DispatchError::ConstraintViolated(TimePoint(p)))
    }
}
