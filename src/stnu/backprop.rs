//! Dynamic controllability by per-label backpropagation from negative nodes.
//!
//! A variant of Morris's 2014 algorithm (see [`super::morris`] for the one-propagation-per-node form) that keeps
//! the network as it is instead of bringing it into normal form, and pays for that with one propagation per
//! kind of negative incoming edge. It is kept as an independent cross-check.
//!
//! Works on the labelled distance graph (see [`super::closure`]) stored as incoming-edge lists.
//! A *negative node* has an incoming edge of negative weight. For each negative node `s`,
//! `backprop(s)` runs Dijkstra backwards from `s`, starting from its negative incoming edges and
//! extending only through non-negative edges. A node `u` reached at distance `d`:
//!
//! - `d >= 0`: the path is summarized by a new ordinary edge `u -> s` with weight `d`, and not
//!   extended further;
//! - `d < 0` and `u` is a negative node: `backprop(u)` runs first, so the new edges into `u` are
//!   available for the extension;
//! - reaching a node whose `backprop` is still running closes a semi-reducible negative cycle:
//!   not DC.
//!
//! The lower-case edge of link `c` is *unsuitable* in a propagation that started with the
//! upper-case edge labelled `c` (combining the two would fake a cycle through nature's own
//! choice). To apply this exactly with one distance per node, every source runs one
//! propagation for its ordinary negative edges and one per incoming upper-case edge.
//!
//! Cost: every node is a source at most once and runs at most `k + 1` propagations. Each source
//! adds at most `n` derived edges, so there are `m' <= m + n^2` edges, and a propagation costs
//! `O(m' log n)`. The worst case is `O(n (k + 1) (m + n^2) log n)`; [`super::morris`] gets
//! Morris's `O(n^3)` from the normal form, in which one propagation per source is exact.

use alloc::collections::BinaryHeap;
use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Reverse;

use super::{Controllability, Stnu};
use crate::budget::Meter;
use crate::{Budget, Error, Ticks};

/// What a labelled distance-graph edge is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// An ordinary (requirement, contingent bound or derived) edge.
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

/// Progress of `backprop` for one node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    /// Never started.
    Unvisited,
    /// On the current recursion stack.
    Active,
    /// Finished without finding a cycle.
    Done,
}

/// Graph and bookkeeping for one check.
struct Graph<'m> {
    /// Incoming edges per node; grows as derived edges are added.
    incoming: Vec<Vec<InEdge>>,
    /// Whether each node has a negative incoming edge. Fixed: derived edges are non-negative.
    negative: Vec<bool>,
    /// `backprop` progress per node.
    status: Vec<Status>,
    /// Step meter shared by all propagations.
    meter: &'m mut Meter,
}

/// Checks dynamic controllability with per-label backpropagation.
///
/// # Errors
///
/// See [`Stnu::check_controllability`].
pub(super) fn run(stnu: &Stnu, budget: &Budget) -> Result<Controllability<()>, Error> {
    let mut meter = budget.meter();
    let mut g = Graph::new(stnu, &mut meter)?;

    for s in 0..g.incoming.len() {
        if g.negative[s] && !g.backprop(s)? {
            return super::morris::explain(stnu, budget).map(Controllability::Uncontrollable);
        }
    }

    Ok(Controllability::Controllable(()))
}

impl<'m> Graph<'m> {
    /// Builds the labelled distance graph of `stnu`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn new(stnu: &Stnu, meter: &'m mut Meter) -> Result<Self, Error> {
        let n = stnu.time_point_count();
        let mut incoming = vec![Vec::new(); n];
        let mut add = |from: usize, to: usize, weight: Ticks, kind: Kind| {
            incoming[to].push(InEdge { from, weight, kind });
        };

        for c in stnu.requirements.constraints() {
            if let Some(hi) = c.hi {
                add(c.from.0, c.to.0, hi, Kind::Ordinary);
            }

            if let Some(lo) = c.lo {
                let w = lo.checked_neg().ok_or(Error::Overflow)?;
                add(c.to.0, c.from.0, w, Kind::Ordinary);
            }
        }

        for (i, link) in stnu.links.iter().enumerate() {
            let (a, x) = (link.activation.0, link.contingent.0);
            add(a, x, link.hi, Kind::Ordinary);
            add(
                x,
                a,
                link.lo.checked_neg().ok_or(Error::Overflow)?,
                Kind::Ordinary,
            );
            add(a, x, link.lo, Kind::Lower(i));
            add(
                x,
                a,
                link.hi.checked_neg().ok_or(Error::Overflow)?,
                Kind::Upper(i),
            );
        }

        meter.spend(crate::stn::steps(n)?)?;
        let negative = incoming
            .iter()
            .map(|edges| edges.iter().any(|e| e.weight < Ticks::ZERO))
            .collect();

        Ok(Self {
            incoming,
            negative,
            status: vec![Status::Unvisited; n],
            meter,
        })
    }

    /// Runs `backprop(source)`; returns false if a semi-reducible negative cycle was found.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn backprop(&mut self, source: usize) -> Result<bool, Error> {
        match self.status[source] {
            Status::Active => return Ok(false),
            Status::Done => return Ok(true),
            Status::Unvisited => {}
        }

        self.status[source] = Status::Active;

        let ordinary: Vec<(usize, Ticks)> = self.incoming[source]
            .iter()
            .filter(|e| e.kind == Kind::Ordinary && e.weight < Ticks::ZERO)
            .map(|e| (e.from, e.weight))
            .collect();
        let upper: Vec<(usize, Ticks, usize)> = self.incoming[source]
            .iter()
            .filter_map(|e| match e.kind {
                Kind::Upper(c) => Some((e.from, e.weight, c)),
                _ => None,
            })
            .collect();

        if !self.propagate(source, &ordinary, None)? {
            return Ok(false);
        }

        for (from, weight, c) in upper {
            if !self.propagate(source, &[(from, weight)], Some(c))? {
                return Ok(false);
            }
        }

        self.status[source] = Status::Done;
        Ok(true)
    }

    /// One backward Dijkstra from `source` over the `initial` edges.
    ///
    /// `label` is the upper-case label of the initial edge, if any; its lower-case edge is then
    /// unsuitable.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn propagate(
        &mut self,
        source: usize,
        initial: &[(usize, Ticks)],
        label: Option<usize>,
    ) -> Result<bool, Error> {
        let mut dist: Vec<Option<Ticks>> = vec![None; self.incoming.len()];
        let mut queue = BinaryHeap::new();

        for &(from, weight) in initial {
            relax(&mut dist, &mut queue, from, weight);
        }

        while let Some(Reverse((d, u))) = queue.pop() {
            if dist[u] != Some(d) {
                continue;
            }

            if d >= Ticks::ZERO {
                self.add_derived(u, source, d);
                continue;
            }

            if self.negative[u] && !self.backprop(u)? {
                return Ok(false);
            }

            self.extend(u, d, label, &mut dist, &mut queue)?;
        }

        Ok(true)
    }

    /// Relaxes every suitable non-negative edge into `u`, which is at distance `d < 0`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn extend(
        &mut self,
        u: usize,
        d: Ticks,
        label: Option<usize>,
        dist: &mut [Option<Ticks>],
        queue: &mut BinaryHeap<Reverse<(Ticks, usize)>>,
    ) -> Result<(), Error> {
        self.meter
            .spend(crate::stn::steps(self.incoming[u].len())?)?;

        for e in &self.incoming[u] {
            let unsuitable = matches!((e.kind, label), (Kind::Lower(c), Some(l)) if c == l);

            if e.weight < Ticks::ZERO || unsuitable {
                continue;
            }

            let candidate = d.checked_add(e.weight).ok_or(Error::Overflow)?;
            relax(dist, queue, e.from, candidate);
        }

        Ok(())
    }

    /// Records the derived ordinary edge `from -> to` with non-negative `weight`.
    fn add_derived(&mut self, from: usize, to: usize, weight: Ticks) {
        if from != to {
            self.incoming[to].push(InEdge {
                from,
                weight,
                kind: Kind::Ordinary,
            });
        }
    }
}

/// Lowers `dist[v]` to `d` and queues `v` if that is an improvement.
fn relax(
    dist: &mut [Option<Ticks>],
    queue: &mut BinaryHeap<Reverse<(Ticks, usize)>>,
    v: usize,
    d: Ticks,
) {
    if dist[v].is_none_or(|old| d < old) {
        dist[v] = Some(d);
        queue.push(Reverse((d, v)));
    }
}
