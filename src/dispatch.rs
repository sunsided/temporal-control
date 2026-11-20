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
//! An edge `x -> y` with weight `w` is a *precedence* edge if `w < 0`, or if `w = 0` and
//! `y < x` (by index): `y` must happen no later than `x`, and for zero weights the index breaks
//! the tie so that zero-weight cycles (simultaneous points) cannot deadlock. Every cycle of
//! precedence edges would be a negative cycle or a zero cycle with strictly decreasing indices,
//! so in a consistent network there is none.
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
//! [`Dispatcher::step`] is `O(N * degree)` per executed point.

#[cfg(feature = "alloc")]
mod network;

use core::fmt;

use crate::{Ticks, TimePoint};

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

/// One adjacency entry, owned by one time point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Entry {
    /// Unused slot.
    Empty,
    /// When the owner happens at `t`: `ub[target] <= t + weight`.
    Upper {
        /// The point whose upper bound tightens.
        target: usize,
        /// Edge weight.
        weight: Ticks,
    },
    /// When the owner happens at `t`: `lb[target] >= t - weight`.
    Lower {
        /// The point whose lower bound tightens.
        target: usize,
        /// Weight of the edge `target -> owner`.
        weight: Ticks,
    },
    /// The owner waits for `contingent` until `activation + delay`.
    Wait {
        /// The contingent point waited for.
        contingent: usize,
        /// Wait length after activation.
        delay: Ticks,
    },
}

/// Fixed-capacity online dispatcher for up to `N` time points and `E` adjacency entries.
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
pub struct Dispatcher<const N: usize, const E: usize> {
    /// Number of time points in use.
    n: usize,
    /// Kind of each point.
    kinds: [PointKind; N],
    /// Lower window bound per point; `None` is `-infinity`.
    lb: [Option<Ticks>; N],
    /// Upper window bound per point; `None` is `+infinity`.
    ub: [Option<Ticks>; N],
    /// Time each point happened, `None` until it happens.
    time: [Option<Ticks>; N],
    /// Per point: outgoing precedence edges whose head has not happened yet.
    pending: [usize; N],
    /// Start of each point's entries in `entries`; the end is the next point's start.
    start: [usize; N],
    /// Number of entries in use.
    m: usize,
    /// Adjacency entries grouped by owner.
    entries: [Entry; E],
    /// Latest time passed to any call; `None` before the first call.
    now: Option<Ticks>,
}

impl<const N: usize, const E: usize> Dispatcher<N, E> {
    /// Validates a dispatchable network and loads it.
    ///
    /// This is the onboard trust boundary: every index, bound and capacity is checked, and
    /// nothing is assumed about how the network was produced.
    ///
    /// # Errors
    ///
    /// - [`DispatchError::CapacityExceeded`] if `points.len() > N` or
    ///   `2 * edges.len() + waits.len() > E`.
    /// - [`DispatchError::InvalidNetwork`] for any malformed input (see its docs).
    pub fn new(
        points: &[PointKind],
        edges: &[DispatchEdge],
        waits: &[Wait],
    ) -> Result<Self, DispatchError> {
        validate(points, edges, waits, N, E)?;

        let mut d = Self {
            n: points.len(),
            kinds: [PointKind::Executable; N],
            lb: [None; N],
            ub: [None; N],
            time: [None; N],
            pending: [0; N],
            start: [0; N],
            m: 0,
            entries: [Entry::Empty; E],
            now: None,
        };

        d.kinds[..points.len()].copy_from_slice(points);
        d.fill_entries(edges, waits);
        Ok(d)
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
            && self.kinds[tp.0] == PointKind::Executable
            && self.time[tp.0].is_none()
            && self.now.is_none_or(|t| t <= now)
            && self.pending[tp.0] == 0
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
                PointKind::Executable => self.ub[p],
                PointKind::Contingent { activation, hi, .. } => {
                    self.time[activation.0].and_then(|a| a.checked_add(hi))
                }
            };

            if deadline.is_some_and(|d| d < now) {
                return Err(match self.kinds[p] {
                    PointKind::Executable => DispatchError::DeadlineMissed(TimePoint(p)),
                    PointKind::Contingent { .. } => {
                        DispatchError::ContingentOutOfBounds(TimePoint(p))
                    }
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
            .filter(|&p| self.kinds[p] == PointKind::Executable && self.time[p].is_none())
            .filter(|&p| self.pending[p] == 0)
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
            .copied()
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

    /// Entries owned by point `p`.
    fn entries_of(&self, p: usize) -> &[Entry] {
        let end = if p + 1 < self.n {
            self.start[p + 1]
        } else {
            self.m
        };

        &self.entries[self.start[p]..end]
    }

    /// Whether every wait of `p` is satisfied at `now`.
    fn waits_satisfied(&self, p: usize, now: Ticks) -> bool {
        self.entries_of(p).iter().all(|e| match *e {
            Entry::Wait { contingent, delay } => self.wait_released(contingent, delay, now),
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

        let PointKind::Contingent { activation, .. } = self.kinds[contingent] else {
            return None;
        };

        self.time[activation.0].and_then(|a| a.checked_add(delay))
    }

    /// Earliest time `>= floor` at which enabled point `p` could execute, if its window allows.
    fn earliest_start(&self, p: usize, floor: Ticks) -> Option<Ticks> {
        let mut t = self.lb[p].map_or(floor, |l| l.max(floor));

        for e in self.entries_of(p) {
            if let Entry::Wait { contingent, delay } = *e {
                t = t.max(self.wait_release_time(contingent, delay)?);
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

        let end = if p + 1 < self.n {
            self.start[p + 1]
        } else {
            self.m
        };

        for i in self.start[p]..end {
            let outcome = match self.entries[i] {
                Entry::Upper { target, weight } => self.tighten_upper(target, t, weight),
                Entry::Lower { target, weight } => self.tighten_lower(p, target, t, weight),
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
            self.pending[target] = self.pending[target].saturating_sub(1);
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

    /// Builds the owner-grouped adjacency from validated input and initializes `pending`.
    fn fill_entries(&mut self, edges: &[DispatchEdge], waits: &[Wait]) {
        let mut degree = [0usize; N];

        for e in edges {
            degree[e.from.0] += 1;
            degree[e.to.0] += 1;
        }

        for w in waits {
            degree[w.waiter.0] += 1;
        }

        let mut next = 0;

        for p in 0..self.n {
            self.start[p] = next;
            next += degree[p];
        }

        self.m = next;
        let mut cursor = self.start;

        for e in edges {
            let (u, v) = (e.from.0, e.to.0);
            push(
                &mut self.entries,
                &mut cursor[u],
                Entry::Upper {
                    target: v,
                    weight: e.weight,
                },
            );
            push(
                &mut self.entries,
                &mut cursor[v],
                Entry::Lower {
                    target: u,
                    weight: e.weight,
                },
            );

            if is_precedence(u, v, e.weight) {
                self.pending[u] += 1;
            }
        }

        for w in waits {
            let entry = Entry::Wait {
                contingent: w.contingent.0,
                delay: w.delay,
            };
            push(&mut self.entries, &mut cursor[w.waiter.0], entry);
        }
    }
}

/// Whether edge `from -> to` with `weight` makes `from` wait for `to` (see the module docs).
fn is_precedence(from: usize, to: usize, weight: Ticks) -> bool {
    weight < Ticks::ZERO || (weight == Ticks::ZERO && to < from)
}

/// Writes `entry` at `*cursor` and advances the cursor.
fn push(entries: &mut [Entry], cursor: &mut usize, entry: Entry) {
    entries[*cursor] = entry;
    *cursor += 1;
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

/// Validates a dispatchable network against capacities `n_cap` and `e_cap`.
///
/// # Errors
///
/// [`DispatchError::CapacityExceeded`] or [`DispatchError::InvalidNetwork`].
fn validate(
    points: &[PointKind],
    edges: &[DispatchEdge],
    waits: &[Wait],
    n_cap: usize,
    e_cap: usize,
) -> Result<(), DispatchError> {
    let entries = edges
        .len()
        .checked_mul(2)
        .and_then(|e| e.checked_add(waits.len()))
        .ok_or(DispatchError::CapacityExceeded)?;

    if points.len() > n_cap || entries > e_cap {
        return Err(DispatchError::CapacityExceeded);
    }

    let n = points.len();
    let valid = points
        .iter()
        .enumerate()
        .all(|(p, k)| valid_point(points, p, *k))
        && edges.iter().all(|e| valid_edge(e, n))
        && waits.iter().all(|w| valid_wait(points, w));

    if !valid {
        return Err(DispatchError::InvalidNetwork);
    }

    Ok(())
}

/// Whether point `p` of kind `kind` is well-formed, including an acyclic activation chain.
fn valid_point(points: &[PointKind], p: usize, kind: PointKind) -> bool {
    let PointKind::Contingent { lo, hi, .. } = kind else {
        return true;
    };

    if lo <= Ticks::ZERO || hi < lo {
        return false;
    }

    let mut current = p;

    for _ in 0..points.len() {
        match points.get(current) {
            Some(PointKind::Contingent { activation, .. }) => current = activation.0,
            Some(PointKind::Executable) => return true,
            None => return false,
        }
    }

    false
}

/// Whether an edge has valid endpoints and is not a negative self-loop.
fn valid_edge(e: &DispatchEdge, n: usize) -> bool {
    e.from.0 < n && e.to.0 < n && (e.from != e.to || e.weight >= Ticks::ZERO)
}

/// Whether a wait goes from an executable point to a contingent point.
fn valid_wait(points: &[PointKind], w: &Wait) -> bool {
    matches!(points.get(w.waiter.0), Some(PointKind::Executable))
        && matches!(
            points.get(w.contingent.0),
            Some(PointKind::Contingent { .. })
        )
}
