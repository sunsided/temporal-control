//! Dynamic controllability by the RUL⁻ algorithm (Cairo, Hunsberger and Rizzi, TIME 2018).
//!
//! # Graph and rules
//!
//! The *LO-graph* has one ordinary edge per requirement bound and the lower-case edge
//! `A -> C: l` of every contingent link `(A, l, u, C)`; the upper-case edges `C -> A: -u` are
//! kept apart, and the link's ordinary bound edges are not part of it. Let `Δ_R = u_R - l_R`.
//! The three rules only generate ordinary edges:
//!
//! - **Relax⁻**: `P -v-> Q -w-> R` gives `P -(v+w)-> R` if `Q` is executable, `R` contingent
//!   and `w < Δ_R`.
//! - **Lower⁻**: `A_Q -q:l-> Q -w-> R` gives `A_Q -(l+w)-> R` if `Q != R` are contingent and
//!   `w < Δ_R`.
//! - **Upper⁻**: `P -v-> R -R:-u-> A_R` gives `P -m-> A_R` with `m = max(v - u, -l)` (not
//!   length-preserving: if `v < Δ_R`, `P` cannot execute before `A_R + l` anyway).
//!
//! # Algorithm (paper's Algorithm 1)
//!
//! One Bellman-Ford pass computes a potential `h` for the LO-graph (`h(P) >= h(Q) - w` for every
//! edge `P -w-> Q`; failure means a negative cycle). Then contingent points are processed with a
//! stack `S` and the set `R` of unprocessed ones, starting with the first link:
//!
//! 1. `CloseRelaxLower(R)`: Dijkstra backwards from `R` over the LO-graph, keyed by
//!    `h(P) + w(P, R)`, applying Relax⁻ and Lower⁻ and adding every improved edge into `R`.
//! 2. `ApplyUpper(R)`: Upper⁻ on every ordinary edge into `R`, adding edges into `A_R`.
//! 3. `UpdatePotential(A_R)`: a Dijkstra-like pass from `A_R` repairs `h` for the new edges;
//!    raising `h(A_R)` itself, or any edge still violating `h`, means a negative LO-cycle: not
//!    DC.
//! 4. If an unprocessed `R'` has an edge `A_R' -s-> R` with `s < Δ_R` (back-propagation from `R`
//!    is blocked by the upper-case edge of `R'`), push `R'`, or answer not DC if `R'` is
//!    already on `S`. Otherwise `R` is done: pop it, and if `S` is empty start the next
//!    unprocessed link.
//!
//! The network is DC iff this finishes. Each round ends by pushing a new point or by popping
//! one for good, so there are at most `2k` rounds.
//!
//! **Reading of the pseudo-code.** Algorithm 2 line 3 seeds the queue with every LO-edge into
//! `R`, which literally includes `R`'s own lower-case edge `A_R -r:l-> R`. Relaxing through it
//! would claim `R <= P + v + l_R` for an edge `P -v-> A_R`, which is unsound (nature may take
//! `u_R`), and no RUL⁻ rule has a lower-case edge as its second edge. So only ordinary edges
//! into `R` seed the queue and enter `w(·, R)`. The pseudo-code's initial `push Z onto S` is
//! read as pushing the first unprocessed contingent point.
//!
//! # Cost
//!
//! `O(n m)` for Bellman-Ford, then at most `2k` rounds. A round touches the `m + 2kn`
//! LO-edges (each round adds at most `n` edges into `R` and `n` into `A_R`) a constant number
//! of times. With a binary heap (one push per relaxation) a round costs
//! `O((m + k n) log n)`, so the total is `O(n m + k (m + k n) log n)`; the paper's
//! `O(m n + k^2 n + k n log n)` needs a heap with `O(1)` decrease-key. Memory: a dense
//! `k * n` table of the edges into contingent points, checked against
//! [`Budget::max_cells`], plus the LO-edges.

use alloc::collections::BinaryHeap;
use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Reverse;

use super::{ContingentLink, Controllability, Stnu};
use crate::budget::Meter;
use crate::stn::steps;
use crate::{Budget, Error, Ticks, TimePoint};

mod rul2021;

/// The LO-graph, the potential and the edges into contingent points during one check.
struct Rul<'m> {
    /// The contingent links, with activation points normalised to be executable.
    links: Vec<ContingentLink>,
    /// Number of nodes: the time points plus one per link with a contingent activation point.
    n: usize,
    /// Link whose contingent point each time point is, if any.
    link_of: Vec<Option<usize>>,
    /// Incoming LO-edges `(tail, weight)` per node; grows with derived edges.
    incoming: Vec<Vec<(usize, Ticks)>>,
    /// Row-major `k * n`: the tightest ordinary edge `P -> C_r` at `[r * n + P]`, self-loops
    /// excluded; `None` is no edge.
    into: Vec<Option<Ticks>>,
    /// Potential: `h[P] >= h[Q] - w` for every LO-edge `P -w-> Q` between rounds.
    h: Vec<Ticks>,
    /// Step meter.
    meter: &'m mut Meter,
}

/// Checks dynamic controllability with RUL⁻; a negative verdict is explained by Morris 2014.
///
/// # Errors
///
/// See [`Stnu::check_controllability_with`].
pub(super) fn run(stnu: &Stnu, budget: &Budget) -> Result<Controllability<()>, Error> {
    let mut meter = budget.meter();
    let mut g = Rul::new(stnu, budget, &mut meter)?;

    if g.init_potential()? && g.process_links()? {
        return Ok(Controllability::Controllable(()));
    }

    super::morris::explain(stnu, budget).map(Controllability::Uncontrollable)
}

/// Checks dynamic controllability with RUL2021 ([`rul2021`]); a negative verdict is explained
/// by Morris 2014.
///
/// # Errors
///
/// See [`Stnu::check_controllability_with`].
pub(super) fn run_2021(stnu: &Stnu, budget: &Budget) -> Result<Controllability<()>, Error> {
    let mut meter = budget.meter();
    let mut g = Rul::new(stnu, budget, &mut meter)?;

    if g.init_potential()? && rul2021::run(&mut g)? {
        return Ok(Controllability::Controllable(()));
    }

    super::morris::explain(stnu, budget).map(Controllability::Uncontrollable)
}

impl<'m> Rul<'m> {
    /// Builds the LO-graph and the table of edges into contingent points.
    ///
    /// The paper assumes executable activation points "without loss of generality". A link
    /// activated by a contingent point `C'` gets a fresh executable node tied to `C'` by two
    /// zero-weight requirement edges instead; under instantaneous reaction that node executes
    /// at the tick `C'` is observed, so DC is preserved.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`] if `k * n` cells exceed the budget,
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn new(stnu: &Stnu, budget: &Budget, meter: &'m mut Meter) -> Result<Self, Error> {
        let points = stnu.time_point_count();
        let mut contingent = vec![false; points];

        for link in &stnu.links {
            contingent[link.contingent.0] = true;
        }

        let mut links = stnu.links.clone();
        let mut ties = Vec::new();

        for link in &mut links {
            if contingent[link.activation.0] {
                let node = points.checked_add(ties.len()).ok_or(Error::Overflow)?;
                ties.push((link.activation.0, node));
                link.activation = TimePoint(node);
            }
        }

        let n = points + ties.len();
        let cells = links
            .len()
            .checked_mul(n)
            .ok_or(Error::MemoryBudgetExceeded)?;
        budget.check_cells(cells)?;
        meter.spend(steps(cells)?.saturating_add(steps(n)?))?;

        let mut g = Self {
            n,
            link_of: vec![None; n],
            incoming: vec![Vec::new(); n],
            into: vec![None; cells],
            h: vec![Ticks::ZERO; n],
            links,
            meter,
        };

        for (r, link) in g.links.iter().enumerate() {
            g.link_of[link.contingent.0] = Some(r);
            g.incoming[link.contingent.0].push((link.activation.0, link.lo));
        }

        for (point, node) in ties {
            g.add_ordinary(point, node, Ticks::ZERO);
            g.add_ordinary(node, point, Ticks::ZERO);
        }

        for c in stnu.requirements.constraints() {
            if let Some(hi) = c.hi {
                g.add_ordinary(c.from.0, c.to.0, hi);
            }

            if let Some(lo) = c.lo {
                g.add_ordinary(c.to.0, c.from.0, lo.checked_neg().ok_or(Error::Overflow)?);
            }
        }

        Ok(g)
    }

    /// Adds the requirement edge `from -> to` to the LO-graph and to the table.
    fn add_ordinary(&mut self, from: usize, to: usize, weight: Ticks) {
        self.incoming[to].push((from, weight));

        if let Some(r) = self.link_of[to] {
            if from != to {
                lower(&mut self.into[r * self.n + from], weight);
            }
        }
    }

    /// Bellman-Ford for the initial potential; false on a negative LO-cycle.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn init_potential(&mut self) -> Result<bool, Error> {
        for _ in 0..self.n {
            self.meter.spend(steps(self.edge_count())?)?;
            let mut changed = false;

            for q in 0..self.n {
                for &(p, w) in &self.incoming[q] {
                    let need = self.h[q].checked_sub(w).ok_or(Error::Overflow)?;

                    if need > self.h[p] {
                        self.h[p] = need;
                        changed = true;
                    }
                }
            }

            if !changed {
                return Ok(true);
            }
        }

        Ok(!self.violates_potential()?)
    }

    /// The main loop of Algorithm 1; false if the network is not DC.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn process_links(&mut self) -> Result<bool, Error> {
        let k = self.links.len();
        let mut unprocessed = vec![true; k];
        let mut on_stack = vec![false; k];
        let mut stack = Vec::new();

        if k > 0 {
            stack.push(0);
            on_stack[0] = true;
        }

        while let Some(&r) = stack.last() {
            if !self.round(r)? {
                return Ok(false);
            }

            if let Some(blocker) = self.blocker(r, &unprocessed) {
                if on_stack[blocker] {
                    return Ok(false);
                }

                stack.push(blocker);
                on_stack[blocker] = true;
                continue;
            }

            unprocessed[r] = false;
            on_stack[r] = false;
            stack.pop();

            if stack.is_empty() {
                if let Some(next) = unprocessed.iter().position(|&u| u) {
                    stack.push(next);
                    on_stack[next] = true;
                }
            }
        }

        Ok(true)
    }

    /// One round for link `r`: generate edges into `C_r` and `A_r`, then repair the potential.
    /// False on a negative LO-cycle.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn round(&mut self, r: usize) -> Result<bool, Error> {
        self.close_relax_lower(r)?;
        self.apply_upper(r)?;
        let activation = self.links[r].activation.0;

        Ok(self.update_potential(activation)? && !self.violates_potential()?)
    }

    /// The first unprocessed link `r'` with an edge `A_r' -s-> C_r` where `s < Δ_r`.
    fn blocker(&self, r: usize, unprocessed: &[bool]) -> Option<usize> {
        let delta = self.delta(r);

        (0..unprocessed.len()).find(|&b| {
            let a = self.links[b].activation.0;
            unprocessed[b] && self.into[r * self.n + a].is_some_and(|s| s < delta)
        })
    }

    /// `CloseRelaxLower`: all edges into `C_r` that Relax⁻ and Lower⁻ generate.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn close_relax_lower(&mut self, r: usize) -> Result<(), Error> {
        let (n, c, delta) = (self.n, self.links[r].contingent.0, self.delta(r));
        let row = r * n;
        let mut queue = BinaryHeap::new();
        let mut settled = vec![false; n];
        let mut improved = vec![false; n];

        for p in 0..n {
            if let Some(w) = self.into[row + p] {
                queue.push(Reverse((self.key(p, w)?, p)));
            }
        }

        while let Some(Reverse((key, q))) = queue.pop() {
            let Some(wq) = self.into[row + q] else {
                return Err(Error::Internal);
            };

            if settled[q] || key != self.key(q, wq)? {
                continue;
            }

            settled[q] = true;

            if wq >= delta {
                continue;
            }

            for (p, v) in self.relax_lower(q, wq, c)? {
                if self.into[row + p].is_none_or(|old| v < old) {
                    self.into[row + p] = Some(v);
                    improved[p] = true;
                    queue.push(Reverse((self.key(p, v)?, p)));
                }
            }
        }

        for p in (0..n).filter(|&p| improved[p]) {
            let w = self.into[row + p].ok_or(Error::Internal)?;
            self.incoming[c].push((p, w));
        }

        Ok(())
    }

    /// `ApplyRelaxLower`: the edges `P -> c` from the path `P -> q -wq-> c`, `wq < Δ`.
    ///
    /// Lower⁻ if `q` is contingent, else Relax⁻ over every LO-edge into `q` not from `c`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn relax_lower(&mut self, q: usize, wq: Ticks, c: usize) -> Result<Vec<(usize, Ticks)>, Error> {
        if let Some(j) = self.link_of[q] {
            let link = &self.links[j];
            let v = link.lo.checked_add(wq).ok_or(Error::Overflow)?;
            let from = link.activation.0;
            return Ok(if from == c {
                Vec::new()
            } else {
                vec![(from, v)]
            });
        }

        self.meter.spend(steps(self.incoming[q].len())?)?;
        self.incoming[q]
            .iter()
            .filter(|&&(p, _)| p != c)
            .map(|&(p, w)| Ok((p, w.checked_add(wq).ok_or(Error::Overflow)?)))
            .collect()
    }

    /// `ApplyUpper`: Upper⁻ on every ordinary edge into `C_r`, adding edges into `A_r`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn apply_upper(&mut self, r: usize) -> Result<(), Error> {
        let link = self.links[r];
        let (delta, neg_lo) = (self.delta(r), link.lo.checked_neg().ok_or(Error::Overflow)?);
        self.meter.spend(steps(self.n)?)?;

        for p in 0..self.n {
            let Some(v) = self.into[r * self.n + p] else {
                continue;
            };

            let m = if v < delta {
                neg_lo
            } else {
                v.checked_sub(link.hi).ok_or(Error::Overflow)?
            };
            self.incoming[link.activation.0].push((p, m));
        }

        Ok(())
    }

    /// `UpdatePotential`: repairs `h` after new edges into `a`; false if `h[a]` itself must rise
    /// (a negative LO-cycle through `a`).
    ///
    /// Largest increase first: every edge not into `a` has non-negative reduced weight under
    /// the old `h`, so increases only shrink along a path, as in Dijkstra.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn update_potential(&mut self, a: usize) -> Result<bool, Error> {
        let old = self.h.clone();
        let mut queue = BinaryHeap::new();
        queue.push((Ticks::ZERO, a));

        while let Some((raise, q)) = queue.pop() {
            if raise != self.h[q].checked_sub(old[q]).ok_or(Error::Overflow)? {
                continue;
            }

            self.meter.spend(steps(self.incoming[q].len())?)?;

            for i in 0..self.incoming[q].len() {
                let (p, w) = self.incoming[q][i];
                let need = self.h[q].checked_sub(w).ok_or(Error::Overflow)?;

                if need <= self.h[p] {
                    continue;
                }

                if p == a {
                    return Ok(false);
                }

                self.h[p] = need;
                queue.push((need.checked_sub(old[p]).ok_or(Error::Overflow)?, p));
            }
        }

        Ok(true)
    }

    /// `NegativeCycle`: whether some LO-edge violates the potential.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn violates_potential(&mut self) -> Result<bool, Error> {
        self.meter.spend(steps(self.edge_count())?)?;

        for q in 0..self.n {
            for &(p, w) in &self.incoming[q] {
                if self.h[q].checked_sub(w).ok_or(Error::Overflow)? > self.h[p] {
                    return Ok(true);
                }
            }
        }

        Ok(false)
    }

    /// `Δ_r = u_r - l_r`, never negative.
    fn delta(&self, r: usize) -> Ticks {
        let link = &self.links[r];
        Ticks(link.hi.0 - link.lo.0)
    }

    /// Dijkstra key of `p` at distance `w` from the current target: `h[p] + w`.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`].
    fn key(&self, p: usize, w: Ticks) -> Result<Ticks, Error> {
        self.h[p].checked_add(w).ok_or(Error::Overflow)
    }

    /// Number of LO-edges.
    fn edge_count(&self) -> usize {
        self.incoming.iter().map(Vec::len).sum()
    }
}

/// Lowers `cell` to `value` if that is tighter.
fn lower(cell: &mut Option<Ticks>, value: Ticks) {
    if cell.is_none_or(|d| value < d) {
        *cell = Some(value);
    }
}
