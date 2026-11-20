//! Dynamic controllability in `O(n^3)` by Morris's backpropagation (Morris 2014).
//!
//! # Graph
//!
//! The labelled distance graph has one ordinary edge per requirement bound, and per contingent
//! link `(A, l, u, C)` the lower-case edge `A -> C: c:l` and the upper-case edge `C -> A: C:-u`.
//! The link's ordinary edges `A -> C: u` and `C -> A: -l` are not part of it, as in the rules of
//! Morris and Muscettola (2005).
//!
//! **Normalisation.** Backpropagation from a node needs its negative incoming edges to be
//! either all ordinary or a single upper-case edge, so that one propagation with one label is
//! exact (see *Unsuitable edges*). As reported by Hunsberger and Posenato (Information and
//! Computation 293, 2023, section 2.1.1), Morris's algorithm establishes this with an `O(n)`
//! pre-processing step. The original paper was not available while writing this; the step
//! used here is: each link `i` gets its own activation node `A'_i`, tied to `A_i` by two
//! zero-weight ordinary edges, and its labelled edges attach to `A'_i`. Then `A'_i` has
//! exactly one negative incoming edge (the upper-case one), and every other node has only
//! ordinary negative incoming edges, because derived edges are never negative. Under
//! instantaneous reaction the split network is DC iff the original is: `A'_i` executes at the
//! tick of `A_i`, also when `A_i` is contingent.
//!
//! # Algorithm
//!
//! A *negative node* has a negative incoming edge. `backprop(s)` runs Dijkstra backwards from
//! `s`, seeded with its negative incoming edges and extending only through non-negative edges.
//! When a node `u` is settled at distance `d`:
//!
//! - `d >= 0`: the path `u ~> s` reduces to a new ordinary edge `u -> s` of weight `d` (by the
//!   no-case and lower-case rules, or for an upper-case seed by the upper-case, cross-case and
//!   label-removal rules, since `d >= 0 > -l`). It is recorded and not extended.
//! - `d < 0` and `u` is a negative node: `backprop(u)` runs first, so that the edges bypassing
//!   `u`'s negative edges exist before the search extends through `u`.
//! - meeting a node whose `backprop` is still on the call stack closes a cycle of negative
//!   reducible paths: a semi-reducible negative cycle, so the network is not DC.
//!
//! **Unsuitable edges.** In a propagation seeded with the upper-case edge of link `c`, the
//! lower-case edge of `c` is skipped: the cross-case rule needs distinct labels, because the
//! same link cannot take its minimum and its maximum duration. Thanks to the normalisation the
//! label is a property of the source (one upper-case seed, or none), so the test is exact with
//! one distance per node.
//!
//! The network is DC iff every negative node's `backprop` completes.
//!
//! # Cost
//!
//! Let `n' = n + k` (at most `2n`) be the node count after normalisation and `m` the number
//! of requirement bounds.
//!
//! 1. Each node runs at most one propagation: a finished `backprop` returns at once, and an
//!    active one ends the check.
//! 2. A propagation settles each node at most once. Extract-min is a linear scan, `O(n')` per
//!    settled node, so `O(n'^2)` per propagation, without a log factor.
//! 3. Each propagation of source `s` adds at most one derived edge per settled node, all into
//!    `s`, so at most `n'^2` derived edges exist in total, and at most `n'` into any node.
//! 4. A settled node `u` with `d < 0` scans `u`'s incoming edges once. Derived edges into `u`
//!    are only added by `backprop(u)` itself, which has finished before the scan, so every
//!    list is scanned at its final length at most once per propagation: `O(m + n'^2)`.
//!
//! So each propagation costs `O(m + n'^2)` and the check `O(n' (m + n'^2)) = O(n m + n^3)`,
//! which is Morris's `O(n^3)` whenever `m = O(n^2)`, i.e. without many parallel constraints
//! on the same pair. Memory: the derived edges and the finished shortest-path trees kept for
//! explanations take `O(m + n'^2)`, the up to `n'` nested searches `O(n'^2)`.
//!
//! # Explanation
//!
//! The cycle consists of one negative path per nested call, from the node that interrupted it
//! to its source. It is collected while the calls unwind. Derived edges are then expanded,
//! through the stored shortest-path tree of the propagation that created them, into the
//! original edges, and the normalisation edges are dropped (they have weight zero and join
//! `A'_i` to `A_i`). The expansion is metered by the step budget; its length is not bounded
//! polynomially here.

use alloc::vec;
use alloc::vec::Vec;

use super::{ContingentId, Controllability, LabelledEdge, SemiReducibleCycle, Stnu};
use crate::budget::Meter;
use crate::stn::{Bound, ConstraintId, CycleEdge, steps};
use crate::{Budget, Error, Ticks};

/// Where an edge of the normalised graph comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Origin {
    /// An original edge of the labelled distance graph.
    Original(LabelledEdge),
    /// One of the two zero-weight edges between a link's own activation node and its
    /// activation point.
    Split,
    /// A non-negative bypass edge `u -> s` found by `backprop(s)`; expands through `s`'s tree.
    Derived,
}

/// An edge `from -> owner`, stored in the owner's incoming list.
#[derive(Debug, Clone, Copy)]
struct InEdge {
    /// Tail of the edge.
    from: usize,
    /// Edge weight.
    weight: Ticks,
    /// What supplied the edge.
    origin: Origin,
}

/// A reference to an edge: `(head, index into the head's incoming list)`.
///
/// Stable because incoming lists only grow at the end.
type EdgeRef = (usize, usize);

/// Progress of `backprop` for one node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    /// Never started.
    Unvisited,
    /// On the current call stack.
    Active,
    /// Finished without finding a cycle.
    Done,
}

/// A semi-reducible negative cycle being collected while `backprop` calls unwind.
struct Trace {
    /// The node whose re-entry closed the cycle; collection stops at its own frame.
    closes_at: usize,
    /// Edges in traversal order, starting at `closes_at`.
    edges: Vec<EdgeRef>,
    /// Whether the frame of `closes_at` has added its path.
    complete: bool,
}

/// Graph and bookkeeping for one check.
struct Graph<'m> {
    /// Incoming edges per node; grows as derived edges are added.
    incoming: Vec<Vec<InEdge>>,
    /// Whether each node has a negative incoming edge. Fixed: derived edges are non-negative.
    negative: Vec<bool>,
    /// `backprop` progress per node.
    status: Vec<Status>,
    /// Shortest-path tree of each finished propagation: `trees[s][u]` is the first edge of the
    /// path `u ~> s`. Empty until `backprop(s)` finishes.
    trees: Vec<Vec<Option<EdgeRef>>>,
    /// Step meter shared by all propagations.
    meter: &'m mut Meter,
}

/// State of one backward Dijkstra.
struct Search {
    /// The node propagated from.
    source: usize,
    /// Link of the source's upper-case seed edge, if it is one; its lower-case edge is
    /// unsuitable.
    label: Option<usize>,
    /// Tentative distance `u ~> source`; `None` is `+infinity`.
    dist: Vec<Option<Ticks>>,
    /// Whether each node is settled.
    settled: Vec<bool>,
    /// First edge of the best path found so far from each node.
    pred: Vec<Option<EdgeRef>>,
}

/// Checks dynamic controllability with Morris's `O(n^3)` backpropagation.
///
/// # Errors
///
/// See [`Stnu::check_controllability`].
pub(super) fn run(stnu: &Stnu, budget: &Budget) -> Result<Controllability<()>, Error> {
    let mut meter = budget.meter();
    let mut g = Graph::new(stnu, budget, &mut meter)?;

    for s in 0..g.incoming.len() {
        if !g.negative[s] {
            continue;
        }

        if let Some(trace) = g.backprop(s)? {
            return g.explain(stnu, &trace).map(Controllability::Uncontrollable);
        }
    }

    Ok(Controllability::Controllable(()))
}

/// The explanation for a network another checker found not DC.
///
/// # Errors
///
/// [`Error::Internal`] if this checker finds the network DC; otherwise as for [`run`].
pub(super) fn explain(stnu: &Stnu, budget: &Budget) -> Result<SemiReducibleCycle, Error> {
    match run(stnu, budget)? {
        Controllability::Uncontrollable(cycle) => Ok(cycle),
        Controllability::Controllable(()) => Err(Error::Internal),
    }
}

impl<'m> Graph<'m> {
    /// Builds the normalised labelled distance graph of `stnu`.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`] if `n'^2` cells exceed the budget (the worst case of
    /// derived edges and trees), [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn new(stnu: &Stnu, budget: &Budget, meter: &'m mut Meter) -> Result<Self, Error> {
        let n = stnu.time_point_count();
        let nodes = n.checked_add(stnu.links.len()).ok_or(Error::Overflow)?;
        budget.check_cells(
            nodes
                .checked_mul(nodes)
                .ok_or(Error::MemoryBudgetExceeded)?,
        )?;

        let mut incoming = vec![Vec::new(); nodes];
        let mut add = |from: usize, to: usize, weight: Ticks, origin: Origin| {
            incoming[to].push(InEdge {
                from,
                weight,
                origin,
            });
        };

        for (i, c) in stnu.requirements.constraints().iter().enumerate() {
            let id = ConstraintId(i);

            if let Some(hi) = c.hi {
                let edge = LabelledEdge::Requirement(CycleEdge::new(id, Bound::Upper));
                add(c.from.0, c.to.0, hi, Origin::Original(edge));
            }

            if let Some(lo) = c.lo {
                let edge = LabelledEdge::Requirement(CycleEdge::new(id, Bound::Lower));
                let w = lo.checked_neg().ok_or(Error::Overflow)?;
                add(c.to.0, c.from.0, w, Origin::Original(edge));
            }
        }

        for (i, link) in stnu.links.iter().enumerate() {
            let (a, own, x) = (link.activation.0, n + i, link.contingent.0);
            let neg_hi = link.hi.checked_neg().ok_or(Error::Overflow)?;
            add(a, own, Ticks::ZERO, Origin::Split);
            add(own, a, Ticks::ZERO, Origin::Split);
            let lower = LabelledEdge::LowerCase(ContingentId(i));
            add(own, x, link.lo, Origin::Original(lower));
            let upper = LabelledEdge::UpperCase(ContingentId(i));
            add(x, own, neg_hi, Origin::Original(upper));
        }

        meter.spend(steps(nodes)?)?;
        let negative = incoming
            .iter()
            .map(|edges| edges.iter().any(|e| e.weight < Ticks::ZERO))
            .collect();

        Ok(Self {
            incoming,
            negative,
            status: vec![Status::Unvisited; nodes],
            trees: vec![Vec::new(); nodes],
            meter,
        })
    }

    /// Runs `backprop(source)`; returns the partial or complete cycle if one was found.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::Overflow`] or [`Error::Internal`].
    fn backprop(&mut self, source: usize) -> Result<Option<Trace>, Error> {
        match self.status[source] {
            Status::Active => {
                return Ok(Some(Trace {
                    closes_at: source,
                    edges: Vec::new(),
                    complete: false,
                }));
            }
            Status::Done => return Ok(None),
            Status::Unvisited => {}
        }

        self.status[source] = Status::Active;
        let mut search = self.seed(source)?;

        while let Some((u, d)) = search.pop(self.meter)? {
            if d >= Ticks::ZERO {
                self.add_derived(u, source, d);
                continue;
            }

            if self.negative[u] {
                if let Some(trace) = self.backprop(u)? {
                    return search.extend_trace(trace, u).map(Some);
                }
            }

            self.extend(&mut search, u, d)?;
        }

        self.status[source] = Status::Done;
        self.trees[source] = search.pred;
        Ok(None)
    }

    /// Starts a search from `source` with its negative incoming edges.
    ///
    /// # Errors
    ///
    /// [`Error::Internal`] if the normalisation invariant (all ordinary, or one upper-case
    /// edge) does not hold.
    fn seed(&self, source: usize) -> Result<Search, Error> {
        let nodes = self.incoming.len();
        let mut search = Search {
            source,
            label: None,
            dist: vec![None; nodes],
            settled: vec![false; nodes],
            pred: vec![None; nodes],
        };
        let mut ordinary = false;

        for (i, e) in self.incoming[source].iter().enumerate() {
            if e.weight >= Ticks::ZERO {
                continue;
            }

            if ordinary || search.label.is_some() {
                // Mixed or repeated upper-case seeds would make one label inexact.
                let upper = matches!(e.origin, Origin::Original(LabelledEdge::UpperCase(_)));

                if upper || search.label.is_some() {
                    return Err(Error::Internal);
                }
            }

            match e.origin {
                Origin::Original(LabelledEdge::UpperCase(c)) => search.label = Some(c.0),
                _ => ordinary = true,
            }

            search.relax(e.from, e.weight, (source, i));
        }

        Ok(search)
    }

    /// Relaxes every suitable non-negative edge into `u`, which is settled at `d < 0`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn extend(&mut self, search: &mut Search, u: usize, d: Ticks) -> Result<(), Error> {
        self.meter.spend(steps(self.incoming[u].len())?)?;

        for (i, e) in self.incoming[u].iter().enumerate() {
            let unsuitable = matches!(
                (e.origin, search.label),
                (Origin::Original(LabelledEdge::LowerCase(c)), Some(l)) if c.0 == l
            );

            if e.weight < Ticks::ZERO || unsuitable {
                continue;
            }

            let candidate = d.checked_add(e.weight).ok_or(Error::Overflow)?;
            search.relax(e.from, candidate, (u, i));
        }

        Ok(())
    }

    /// Records the derived ordinary edge `from -> to` with non-negative `weight`.
    fn add_derived(&mut self, from: usize, to: usize, weight: Ticks) {
        if from != to {
            self.incoming[to].push(InEdge {
                from,
                weight,
                origin: Origin::Derived,
            });
        }
    }

    /// Expands a complete trace into original edges and checks that it is negative.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::Overflow`], or [`Error::Internal`] if the trace
    /// is incomplete, empty or not negative.
    fn explain(&mut self, stnu: &Stnu, trace: &Trace) -> Result<SemiReducibleCycle, Error> {
        if !trace.complete {
            return Err(Error::Internal);
        }

        let mut pending: Vec<EdgeRef> = trace.edges.iter().rev().copied().collect();
        let mut edges = Vec::new();
        let mut weight = Ticks::ZERO;

        while let Some((head, i)) = pending.pop() {
            self.meter.spend(1)?;
            let e = self.incoming[head][i];

            match e.origin {
                Origin::Original(edge) => {
                    weight = weight
                        .checked_add(edge.resolve(stnu)?.2)
                        .ok_or(Error::Overflow)?;
                    edges.push(edge);
                }
                Origin::Split => {}
                Origin::Derived => {
                    let path = walk(&self.trees[head], e.from, head)?;
                    pending.extend(path.into_iter().rev());
                }
            }
        }

        if edges.is_empty() || weight >= Ticks::ZERO {
            return Err(Error::Internal);
        }

        Ok(SemiReducibleCycle::new(edges, weight))
    }
}

impl Search {
    /// Settles the closest unsettled node and returns it with its distance.
    ///
    /// A linear scan, `O(n')` steps; ties go to the lowest index, so results are
    /// deterministic.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn pop(&mut self, meter: &mut Meter) -> Result<Option<(usize, Ticks)>, Error> {
        meter.spend(steps(self.dist.len())?)?;
        let mut best: Option<(usize, Ticks)> = None;

        let open = self.dist.iter().zip(&self.settled).enumerate();

        for (u, (d, settled)) in open {
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

    /// Lowers the distance of `v` to `d` via `edge` if that is an improvement.
    fn relax(&mut self, v: usize, d: Ticks, edge: EdgeRef) {
        if !self.settled[v] && self.dist[v].is_none_or(|old| d < old) {
            self.dist[v] = Some(d);
            self.pred[v] = Some(edge);
        }
    }

    /// Appends this search's path from `u` (where a nested call found a cycle) to the source,
    /// unless the trace is already complete.
    ///
    /// # Errors
    ///
    /// [`Error::Internal`] if the path is broken.
    fn extend_trace(&self, mut trace: Trace, u: usize) -> Result<Trace, Error> {
        if trace.complete {
            return Ok(trace);
        }

        trace.edges.extend(walk(&self.pred, u, self.source)?);
        trace.complete = self.source == trace.closes_at;
        Ok(trace)
    }
}

/// The edges of the path `from ~> target` in a shortest-path tree, in traversal order.
///
/// # Errors
///
/// [`Error::Internal`] if the tree has no such path.
fn walk(tree: &[Option<EdgeRef>], from: usize, target: usize) -> Result<Vec<EdgeRef>, Error> {
    let mut path = Vec::new();
    let mut at = from;

    while path.len() < tree.len() {
        let (head, i) = tree.get(at).copied().flatten().ok_or(Error::Internal)?;
        path.push((head, i));

        if head == target {
            return Ok(path);
        }

        at = head;
    }

    Err(Error::Internal)
}
