//! The RUL2021 algorithm (Hunsberger and Posenato, AAAI 2022), Algorithms 1 to 7.
//!
//! Same graph, potential and rules as RUL⁻ ([`super`]), but it processes each upper-case
//! edge `E = (C, C:-y, A)` once, recursively, and inserts far fewer edges:
//!
//! - **Phase 1** (`phase1`, `apRL`): Dijkstra backwards from `C` over the LO-graph, keyed by
//!   `h(X) + δ(X, C)`, applying Relax⁻ and Lower⁻ but only *accumulating* the distances
//!   `dist[X]`, never inserting edges. A node settled at `δ < Δ_C = y - x` is either `C`
//!   itself (a *CC loop*: negative means not DC, otherwise remembered), the activation point of
//!   an unstarted upper-case edge (collected; propagation stops there), of a started one (a
//!   cycle of interruptions: not DC), or extended.
//! - **Interruptions** (`RULbp`): the collected upper-case edges are processed recursively;
//!   then phase 1 *resumes* from their activation points (with the updated potential) instead
//!   of restarting from scratch.
//! - **CC loops** (`fwdPropNDC`): if a non-negative loop `C ~> C` shorter than `Δ_C` was seen,
//!   a forward Dijkstra from `C` over the LO-graph, restricted to nodes with `dist < Δ_C`,
//!   looks for a negative path that reduces away the lower-case edge of `E`: if one exists,
//!   not DC. This replaces RUL⁻'s non-length-preserving Upper⁻ rule.
//! - **Phase 2**: only the length-preserving Upper⁻ edges `X -> A: dist[X] - y` for
//!   `Δ_C <= dist[X]` are inserted, then the potential is repaired (`UpdPF`).
//!
//! Each upper-case edge is processed once, so there are `k` recursive calls; with a binary heap
//! the cost is `O(n m + k (m + k n) log n)` like RUL⁻'s, but with far fewer inserted edges in
//! practice. Normalisation (executable activation points) and the seeding with ordinary edges
//! only are as in [`super`]; Algorithm 6 line 11 seeds with `E^o` edges into `C`, which
//! confirms that reading. An activation point of several links counts as unstarted if any of
//! its upper-case edges is, and as started if any is (the paper assumes one link per point).

use alloc::collections::BinaryHeap;
use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Reverse;

use super::Rul;
use crate::stn::steps;
use crate::{Error, Ticks};

/// Processing status of an upper-case edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    /// Not yet started (`nYet`).
    NotYet,
    /// Started, on the recursion stack (`std`).
    Started,
    /// Reduced away (`done`).
    Done,
}

/// Local state of one `RULbp` call (the paper's `loc`).
struct Local {
    /// Whether a loop `C ~> C` of length in `[0, Δ_C)` was found.
    cc_loop: bool,
    /// Shortest distance found so far from each node to `C`; `None` is `+infinity`.
    dist: Vec<Option<Ticks>>,
    /// Activation points of unstarted upper-case edges met in the last phase 1; no
    /// duplicates.
    unstarted: Vec<usize>,
}

/// Global state of a run (the paper's `glo` plus the graph).
pub(super) struct Rul2021<'g, 'm> {
    /// Graph, potential and meter, shared with RUL⁻.
    g: &'g mut Rul<'m>,
    /// Status per upper-case edge (link).
    status: Vec<Status>,
    /// Links activated by each node.
    activates: Vec<Vec<usize>>,
}

/// Runs RUL2021 on a graph with a valid initial potential; false if not DC.
///
/// # Errors
///
/// [`Error::BudgetExhausted`] or [`Error::Overflow`].
pub(super) fn run(g: &mut Rul<'_>) -> Result<bool, Error> {
    let k = g.links.len();
    let mut activates = vec![Vec::new(); g.n];

    for (r, link) in g.links.iter().enumerate() {
        activates[link.activation.0].push(r);
    }

    let mut s = Rul2021 {
        g,
        status: vec![Status::NotYet; k],
        activates,
    };

    for r in 0..k {
        if !s.process(r)? {
            return Ok(false);
        }
    }

    Ok(true)
}

impl Rul2021<'_, '_> {
    /// `RULbp`: reduces away the upper-case edge of link `r`; false if not DC.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn process(&mut self, r: usize) -> Result<bool, Error> {
        match self.status[r] {
            Status::Started => return Ok(false),
            Status::Done => return Ok(true),
            Status::NotYet => {}
        }

        self.status[r] = Status::Started;
        let n = self.g.n;
        let mut loc = Local {
            cc_loop: false,
            dist: vec![None; n],
            unstarted: Vec::new(),
        };
        let mut queue = self.initial_queue(r)?;

        loop {
            if !self.phase1(r, &mut queue, &mut loc)? {
                return Ok(false);
            }

            if loc.unstarted.is_empty() {
                break;
            }

            if !self.resume_after_interruptions(&mut queue, &mut loc)? {
                return Ok(false);
            }
        }

        if loc.cc_loop && self.lower_case_reducible(r, &loc.dist)? {
            return Ok(false);
        }

        if !self.phase2(r, &loc.dist)? {
            return Ok(false);
        }

        self.status[r] = Status::Done;
        Ok(true)
    }

    /// The phase-1 queue: every tail of an ordinary edge into `C_r`, keyed `h(X) + w`.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`].
    fn initial_queue(&self, r: usize) -> Result<BinaryHeap<Reverse<(Ticks, usize)>>, Error> {
        let n = self.g.n;
        let mut queue = BinaryHeap::new();

        for x in 0..n {
            if let Some(w) = self.g.into[r * n + x] {
                queue.push(Reverse((self.g.key(x, w)?, x)));
            }
        }

        Ok(queue)
    }

    /// Processes the upper-case edges phase 1 stopped at, then reseeds the queue with their
    /// activation points under the updated potential; false if not DC.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn resume_after_interruptions(
        &mut self,
        queue: &mut BinaryHeap<Reverse<(Ticks, usize)>>,
        loc: &mut Local,
    ) -> Result<bool, Error> {
        let points = core::mem::take(&mut loc.unstarted);

        for &x in &points {
            for r in self.activates[x].clone() {
                if self.status[r] == Status::NotYet && !self.process(r)? {
                    return Ok(false);
                }
            }
        }

        queue.clear();

        for &x in &points {
            let d = loc.dist[x].ok_or(Error::Internal)?;
            queue.push(Reverse((self.g.key(x, d)?, x)));
            loc.dist[x] = None;
        }

        Ok(true)
    }

    /// `phase1`: backward Dijkstra from `C_r` accumulating `loc.dist`; false if not DC.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn phase1(
        &mut self,
        r: usize,
        queue: &mut BinaryHeap<Reverse<(Ticks, usize)>>,
        loc: &mut Local,
    ) -> Result<bool, Error> {
        let (n, c, delta) = (self.g.n, self.g.links[r].contingent.0, self.g.delta(r));

        while let Some(Reverse((key, x))) = queue.pop() {
            let d = key.checked_sub(self.g.h[x]).ok_or(Error::Overflow)?;

            if loc.dist[x].is_some_and(|old| old <= d) {
                continue;
            }

            loc.dist[x] = Some(d);

            if d >= delta {
                continue;
            }

            if x == c {
                if d < Ticks::ZERO {
                    return Ok(false);
                }

                loc.cc_loop = true;
                continue;
            }

            let links = &self.activates[x];

            if links.iter().any(|&l| self.status[l] == Status::Started) {
                return Ok(false);
            }

            if links.iter().any(|&l| self.status[l] == Status::NotYet) {
                if !loc.unstarted.contains(&x) {
                    loc.unstarted.push(x);
                }

                continue;
            }

            for (w, v) in self.relax_lower(x, d)? {
                if self.g.into[r * n + w].is_none_or(|old| v < old) {
                    queue.push(Reverse((self.g.key(w, v)?, w)));
                }
            }
        }

        Ok(true)
    }

    /// `apRL`: Lower⁻ from a contingent `v`, else Relax⁻ over the ordinary edges into `v`
    /// (the tail may be `C` itself, which is how CC loops are found).
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn relax_lower(&mut self, v: usize, d: Ticks) -> Result<Vec<(usize, Ticks)>, Error> {
        if let Some(j) = self.g.link_of[v] {
            let link = &self.g.links[j];
            let w = link.lo.checked_add(d).ok_or(Error::Overflow)?;
            return Ok(vec![(link.activation.0, w)]);
        }

        self.g.meter.spend(steps(self.g.incoming[v].len())?)?;
        self.g.incoming[v]
            .iter()
            .map(|&(p, w)| Ok((p, w.checked_add(d).ok_or(Error::Overflow)?)))
            .collect()
    }

    /// `fwdPropNDC`: whether a negative LO-path from `C_r`, through nodes with
    /// `dist < Δ_r`, reduces away the lower-case edge of `r`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn lower_case_reducible(&mut self, r: usize, dist: &[Option<Ticks>]) -> Result<bool, Error> {
        let (n, c, delta) = (self.g.n, self.g.links[r].contingent.0, self.g.delta(r));
        let outgoing = self.outgoing()?;
        let mut best: Vec<Option<Ticks>> = vec![None; n];
        let mut queue = BinaryHeap::new();
        let start = Ticks::ZERO
            .checked_sub(self.g.h[c])
            .ok_or(Error::Overflow)?;
        queue.push(Reverse((start, c)));

        while let Some(Reverse((key, x))) = queue.pop() {
            if best[x].is_some_and(|b| b <= key) {
                continue;
            }

            best[x] = Some(key);

            if dist[x].is_none_or(|d| d >= delta) {
                continue;
            }

            let dcx = key.checked_add(self.g.h[x]).ok_or(Error::Overflow)?;

            if dcx < Ticks::ZERO {
                return Ok(true);
            }

            self.g.meter.spend(steps(outgoing[x].len())?)?;

            for &(y, w) in &outgoing[x] {
                let next = dcx.checked_add(w).ok_or(Error::Overflow)?;
                let next_key = next.checked_sub(self.g.h[y]).ok_or(Error::Overflow)?;
                queue.push(Reverse((next_key, y)));
            }
        }

        Ok(false)
    }

    /// Phase 2: the length-preserving Upper⁻ edges into `A_r`, then `UpdPF`; false if the
    /// potential cannot be repaired (not DC).
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn phase2(&mut self, r: usize, dist: &[Option<Ticks>]) -> Result<bool, Error> {
        let link = self.g.links[r];
        let delta = self.g.delta(r);
        let mut inserted = false;

        for (tail, d) in dist.iter().enumerate() {
            let Some(d) = *d else { continue };

            if tail != link.contingent.0 && d >= delta {
                let w = d.checked_sub(link.hi).ok_or(Error::Overflow)?;
                self.g.incoming[link.activation.0].push((tail, w));
                inserted = true;
            }
        }

        Ok(!inserted || self.g.update_potential(link.activation.0)?)
    }

    /// Outgoing LO-edges per node, built from the incoming lists.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn outgoing(&mut self) -> Result<Vec<Vec<(usize, Ticks)>>, Error> {
        self.g.meter.spend(steps(self.g.edge_count())?)?;
        let mut out = vec![Vec::new(); self.g.n];

        for (q, edges) in self.g.incoming.iter().enumerate() {
            for &(p, w) in edges {
                out[p].push((q, w));
            }
        }

        Ok(out)
    }
}
