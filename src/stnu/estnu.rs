//! A dispatchable extended STNU by Morris's dispatchable variant of his 2014 algorithm.
//!
//! Morris (2014) noted that his `O(n^3)` DC check can also insert the edges it otherwise only
//! computes on the way, both ordinary edges and waits, and that for a DC input the result is
//! a dispatchable extended STNU (ESTNU): every STN projection is dispatchable. This module
//! follows CSTNU-Tool's `applyMorris2014Dispatchable` and
//! `morris2014DispatchableDCBackpropagation` (read, not copied), which, unlike the plain check
//! in [`super::morris`], do *not* use the normal form: its rigid tie edges would stay in the
//! output and break the dispatchability of the projections.
//!
//! # Passes
//!
//! Without the normal form a node can have several kinds of negative incoming edges, so each
//! negative node `X` runs one pass per link `c` that it activates, then one ordinary pass:
//!
//! - **Upper-case pass for `c`.** Seeded with the upper-case edge `C_c -> X: C:-u_c`. A node `U`
//!   settled at distance `d`:
//!   - `d >= 0`: the ordinary edge `U -> X: d` (upper case plus label removal); not extended.
//!   - `d < 0`, `U != C_c`, `d >= -l_c`: the ordinary negative edge `U -> X: d` (label removal);
//!     not extended, because the ordinary pass that follows picks it up as a seed.
//!   - `d < -l_c`: the wait `U -> X: C:d` ("`U` waits `-d` after `X` unless `C` happened");
//!     extended. The lower-case edge of `c` is unsuitable here.
//! - **Ordinary pass.** Seeded with every negative ordinary edge into `X`, including those the
//!   upper-case passes just added. A node `U` settled at `d`: `d >= 0` gives `U -> X: d`, not
//!   extended; `d < 0` gives `U -> X: d` (unless `U` is a contingent point activated by `X`,
//!   where that edge only restates the link) and is extended.
//!
//! As in the plain check, a negative node met at `d < 0` is processed first (recursively), and
//! meeting a node still being processed means the network is not DC.
//!
//! # Cost
//!
//! There are at most `n + k` passes. Extract-min is a linear scan and every edge list is
//! scanned at most once per pass, so a pass costs `O(n^2 + m')`, where `m' <= m + n (n + k)`
//! counts the original edges plus at most one inserted edge per improvement of a
//! `(node, source)` pair and pass. Total: `O((n + k)(m + n^2 + n k)) = O(n m + n^3)` for
//! `k <= n`, plus `n^2 + n k` cells for the inserted ordinary edges and waits, checked against
//! [`Budget::max_cells`].

use alloc::vec;
use alloc::vec::Vec;

use super::Stnu;
use crate::budget::Meter;
use crate::dispatch::{DispatchEdge, DispatchableNetwork, PointKind, Wait};
use crate::stn::steps;
use crate::{Budget, Error, Ticks, TimePoint};

/// The inserted constraints of a DC network: everything a dispatcher needs besides the links.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Estnu {
    /// Number of time points.
    pub(super) n: usize,
    /// Number of contingent links.
    pub(super) k: usize,
    /// Row-major `n * n`: tightest ordinary edge `u -> v`, requirements included; diagonal
    /// unused (`None`).
    pub(super) ordinary: Vec<Option<Ticks>>,
    /// Row-major `n * k`: tightest wait `u -> A_c: C:d` with `d < -l_c`; `None` if none.
    pub(super) waits: Vec<Option<Ticks>>,
}

/// What a labelled distance-graph edge is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// An ordinary edge, original or inserted.
    Ordinary,
    /// The lower-case edge `A_c -> C_c` of link `c`.
    Lower(usize),
    /// The upper-case edge `C_c -> A_c` of link `c`.
    Upper(usize),
}

/// An edge `from -> owner`, stored in the owner's incoming list.
#[derive(Debug, Clone, Copy)]
struct InEdge {
    /// Tail of the edge.
    from: usize,
    /// Edge weight.
    weight: Ticks,
    /// Edge kind.
    kind: Kind,
}

/// Progress of the passes of one node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    /// Never started.
    Unvisited,
    /// On the current call stack.
    Active,
    /// All passes finished.
    Done,
}

/// Graph, output and bookkeeping for one run.
struct Graph<'s, 'm> {
    /// The network.
    stnu: &'s Stnu,
    /// Incoming edges per node; grows with inserted ordinary edges.
    incoming: Vec<Vec<InEdge>>,
    /// Whether each node has a negative incoming ordinary or upper-case edge initially.
    negative: Vec<bool>,
    /// Pass progress per node.
    status: Vec<Status>,
    /// Links activated by each node.
    activates: Vec<Vec<usize>>,
    /// The output being built.
    out: Estnu,
    /// Step meter.
    meter: &'m mut Meter,
}

/// One backward Dijkstra.
struct Search {
    /// Tentative distance to the source; `None` is `+infinity`.
    dist: Vec<Option<Ticks>>,
    /// Whether each node is settled.
    settled: Vec<bool>,
}

/// Runs Morris's dispatchable variant; `None` if the network is not DC.
///
/// # Errors
///
/// [`Error::MemoryBudgetExceeded`], [`Error::BudgetExhausted`] or [`Error::Overflow`].
pub(super) fn run(stnu: &Stnu, budget: &Budget) -> Result<Option<Estnu>, Error> {
    let mut meter = budget.meter();
    let mut g = Graph::new(stnu, budget, &mut meter)?;

    for x in 0..g.incoming.len() {
        if g.negative[x] && !g.process(x)? {
            return Ok(None);
        }
    }

    Ok(Some(g.out))
}

impl<'s, 'm> Graph<'s, 'm> {
    /// Loads the labelled distance graph (requirements, lower- and upper-case edges).
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`], [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn new(stnu: &'s Stnu, budget: &Budget, meter: &'m mut Meter) -> Result<Self, Error> {
        let (points, links) = (stnu.time_point_count(), stnu.links.len());
        let nn = points
            .checked_mul(points)
            .ok_or(Error::MemoryBudgetExceeded)?;
        let nk = points
            .checked_mul(links)
            .ok_or(Error::MemoryBudgetExceeded)?;
        budget.check_cells(nn.checked_add(nk).ok_or(Error::MemoryBudgetExceeded)?)?;
        meter.spend(steps(nn)?.saturating_add(steps(nk)?))?;

        let mut g = Self {
            stnu,
            incoming: vec![Vec::new(); points],
            negative: vec![false; points],
            status: vec![Status::Unvisited; points],
            activates: vec![Vec::new(); points],
            out: Estnu {
                n: points,
                k: links,
                ordinary: vec![None; nn],
                waits: vec![None; nk],
            },
            meter,
        };

        g.load_requirements()?;
        g.load_links()?;
        g.negative = g
            .incoming
            .iter()
            .map(|edges| {
                edges
                    .iter()
                    .any(|e| e.weight < Ticks::ZERO && !matches!(e.kind, Kind::Lower(_)))
            })
            .collect();

        Ok(g)
    }

    /// Inserts the edges of the requirement bounds.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a lower bound cannot be negated.
    fn load_requirements(&mut self) -> Result<(), Error> {
        for c in self.stnu.requirements.constraints() {
            if let Some(hi) = c.hi {
                self.insert_ordinary(c.from.0, c.to.0, hi);
            }

            if let Some(lo) = c.lo {
                let w = lo.checked_neg().ok_or(Error::Overflow)?;
                self.insert_ordinary(c.to.0, c.from.0, w);
            }
        }

        Ok(())
    }

    /// Adds the lower- and upper-case edge of every link.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a bound cannot be negated.
    fn load_links(&mut self) -> Result<(), Error> {
        for (i, link) in self.stnu.links.iter().enumerate() {
            let (a, x) = (link.activation.0, link.contingent.0);
            self.activates[a].push(i);
            self.incoming[x].push(InEdge {
                from: a,
                weight: link.lo,
                kind: Kind::Lower(i),
            });
            self.incoming[a].push(InEdge {
                from: x,
                weight: link.hi.checked_neg().ok_or(Error::Overflow)?,
                kind: Kind::Upper(i),
            });
        }

        Ok(())
    }

    /// Runs every pass of `x` (recursing into negative nodes); false if not DC.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn process(&mut self, x: usize) -> Result<bool, Error> {
        match self.status[x] {
            Status::Active => return Ok(false),
            Status::Done => return Ok(true),
            Status::Unvisited => {}
        }

        self.status[x] = Status::Active;

        for c in self.activates[x].clone() {
            if !self.pass(x, Some(c))? {
                return Ok(false);
            }
        }

        if !self.pass(x, None)? {
            return Ok(false);
        }

        self.status[x] = Status::Done;
        Ok(true)
    }

    /// One pass of `x`: upper-case for `label`, or ordinary for `None`; false if not DC.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn pass(&mut self, x: usize, label: Option<usize>) -> Result<bool, Error> {
        let mut search = self.seed(x, label)?;

        while let Some((u, d)) = search.pop(self.meter)? {
            if d >= Ticks::ZERO {
                self.insert_ordinary(u, x, d);
                continue;
            }

            if self.negative[u] && !self.process(u)? {
                return Ok(false);
            }

            if !self.record_negative(x, label, u, d) {
                continue;
            }

            self.extend(&mut search, x, label, u, d)?;
        }

        Ok(true)
    }

    /// The seeds of a pass of `x`.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`].
    fn seed(&self, x: usize, label: Option<usize>) -> Result<Search, Error> {
        let n = self.incoming.len();
        let mut search = Search {
            dist: vec![None; n],
            settled: vec![false; n],
        };

        if let Some(c) = label {
            let link = &self.stnu.links[c];
            let w = link.hi.checked_neg().ok_or(Error::Overflow)?;
            search.relax(link.contingent.0, w);
            return Ok(search);
        }

        for e in &self.incoming[x] {
            if e.kind == Kind::Ordinary && e.weight < Ticks::ZERO {
                search.relax(e.from, e.weight);
            }
        }

        Ok(search)
    }

    /// Inserts what a node `u` settled at `d < 0` in a pass of `x` implies; returns whether
    /// the pass extends through `u`.
    fn record_negative(&mut self, x: usize, label: Option<usize>, u: usize, d: Ticks) -> bool {
        let Some(c) = label else {
            if !self.is_contingent_of(u, x) {
                self.insert_ordinary(u, x, d);
            }

            return true;
        };

        let link = self.stnu.links[c];

        if u == link.contingent.0 {
            return true;
        }

        if d.0 >= -link.lo.0 {
            self.insert_ordinary(u, x, d);
            return false;
        }

        lower(&mut self.out.waits[u * self.out.k + c], d);
        true
    }

    /// Relaxes every suitable non-negative edge into `u`, settled at `d < 0`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn extend(
        &mut self,
        search: &mut Search,
        x: usize,
        label: Option<usize>,
        u: usize,
        d: Ticks,
    ) -> Result<(), Error> {
        self.meter.spend(steps(self.incoming[u].len())?)?;

        for e in &self.incoming[u] {
            let unsuitable = match e.kind {
                Kind::Upper(_) => true,
                Kind::Lower(c) => label == Some(c) && e.from == x,
                Kind::Ordinary => false,
            };

            if unsuitable || e.weight < Ticks::ZERO {
                continue;
            }

            search.relax(e.from, d.checked_add(e.weight).ok_or(Error::Overflow)?);
        }

        Ok(())
    }

    /// Whether `u` is the contingent point of a link activated by `x`.
    fn is_contingent_of(&self, u: usize, x: usize) -> bool {
        self.activates[x]
            .iter()
            .any(|&c| self.stnu.links[c].contingent.0 == u)
    }

    /// Records the ordinary edge `from -> to` if it is tighter than the current one.
    fn insert_ordinary(&mut self, from: usize, to: usize, weight: Ticks) {
        if from == to {
            if weight < Ticks::ZERO {
                // A negative self-loop: kept as an edge so the seeding of `to` finds it.
                self.incoming[to].push(InEdge {
                    from,
                    weight,
                    kind: Kind::Ordinary,
                });
            }

            return;
        }

        let cell = &mut self.out.ordinary[from * self.out.n + to];

        if cell.is_none_or(|w| weight < w) {
            *cell = Some(weight);
            self.incoming[to].push(InEdge {
                from,
                weight,
                kind: Kind::Ordinary,
            });
        }
    }
}

impl Search {
    /// Settles the closest unsettled node; linear scan, lowest index on ties.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn pop(&mut self, meter: &mut Meter) -> Result<Option<(usize, Ticks)>, Error> {
        meter.spend(steps(self.dist.len())?)?;
        let mut best: Option<(usize, Ticks)> = None;

        for (u, (d, settled)) in self.dist.iter().zip(&self.settled).enumerate() {
            if let (Some(d), false) = (*d, *settled) {
                if best.is_none_or(|(_, b)| d < b) {
                    best = Some((u, d));
                }
            }
        }

        if let Some((u, _)) = best {
            self.settled[u] = true;
        }

        Ok(best)
    }

    /// Lowers the distance of `v` to `d` if that is an improvement.
    fn relax(&mut self, v: usize, d: Ticks) {
        if !self.settled[v] && self.dist[v].is_none_or(|old| d < old) {
            self.dist[v] = Some(d);
        }
    }
}

/// Lowers `cell` to `value` if that is tighter.
fn lower(cell: &mut Option<Ticks>, value: Ticks) {
    if cell.is_none_or(|d| value < d) {
        *cell = Some(value);
    }
}

impl Estnu {
    /// The dispatchable network: every inserted ordinary edge, and every wait of an
    /// executable point that its parallel ordinary edge does not already imply.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`], or [`Error::Overflow`] if a wait cannot be negated.
    pub(super) fn into_network(self, stnu: &Stnu) -> Result<DispatchableNetwork, Error> {
        let points = point_kinds(stnu)?;
        let mut edges = Vec::new();

        for (i, w) in self.ordinary.iter().enumerate() {
            if let Some(weight) = *w {
                edges.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
                edges.push(DispatchEdge {
                    from: TimePoint(i / self.n),
                    to: TimePoint(i % self.n),
                    weight,
                });
            }
        }

        let mut waits = Vec::new();

        for (c, link) in stnu.links.iter().enumerate() {
            let a = link.activation.0;

            for u in (0..self.n).filter(|&u| points[u] == PointKind::Executable) {
                let Some(d) = self.waits[u * self.k + c] else {
                    continue;
                };

                if self.ordinary[u * self.n + a].is_none_or(|w| d < w) {
                    waits.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
                    waits.push(Wait {
                        waiter: TimePoint(u),
                        contingent: link.contingent,
                        delay: d.checked_neg().ok_or(Error::Overflow)?,
                    });
                }
            }
        }

        Ok(DispatchableNetwork::new(points, edges, waits))
    }
}

/// The kind of every time point of `stnu`, for a dispatcher.
///
/// # Errors
///
/// [`Error::OutOfMemory`].
pub(super) fn point_kinds(stnu: &Stnu) -> Result<Vec<PointKind>, Error> {
    let mut points = Vec::new();
    points
        .try_reserve_exact(stnu.time_point_count())
        .map_err(|_| Error::OutOfMemory)?;
    points.resize(stnu.time_point_count(), PointKind::Executable);

    for link in &stnu.links {
        points[link.contingent.0] = PointKind::Contingent {
            activation: link.activation,
            lo: link.lo,
            hi: link.hi,
        };
    }

    Ok(points)
}
