//! Fewer edges and waits: `minDispESTNU` (Hunsberger and Posenato, ICAPS 2024).
//!
//! Input is the dispatchable ESTNU of [`super::estnu`]. The four steps of the paper's
//! Algorithm 1:
//!
//! 1. **Stand-in edges** (`genStandIns`, Algorithm 2): ordinary edges that are entailed across
//!    all STN projections but not explicit. Per link `(A, x, y, C)`: `A -> C: y` and
//!    `C -> A: -x`; per wait `V -> A: C:-v` with `-v < -x`: `V -> A: -x`. Then, for at most `k`
//!    rounds and with `D` the all-pairs distances of the ordinary edges plus stand-ins, the
//!    *VAC rule*: `V -> C: y - v` if tighter than `D(V, C)`. Weak waits (`-v >= -x`) become
//!    ordinary edges and misleading ones (`-v < -y`) are clamped to `-y` first; Morris's
//!    dispatchable variant produces neither, but the step is kept as in the paper.
//! 2. **STN dispatchability** of the ordinary edges plus stand-ins:
//!    [`MinimalNetwork::dispatchable`](crate::MinimalNetwork::dispatchable) (minimal network
//!    and dominated-edge filtering, rigid components tied to a leader).
//! 3. **Drop stand-ins** that survived the filtering: other edges entail them.
//! 4. **Drop dominated waits** (`markWaits`): a wait `V -> A: C:-v` goes if an ordinary path
//!    implies it (`D(V, A) <= -v`), if `V` must follow `C` anyway (`D(V, C) < 0`, dominated by
//!    the upper-case edge), or if another wait `U -> A: C:-u` with `D(V, U) < 0` and
//!    `D(V, U) - u <= -v` implies it.
//!
//! **Deviations.** The paper's executor `RTE*` propagates through labelled edges and waits;
//! this crate's [`Dispatcher`](crate::Dispatcher) only propagates along ordinary edges. So:
//!
//! - The *diamond* stand-ins of step 1 (`V -> W: max(-ω, -v) + δ` for `V, A, C, W` with
//!   `ω = D(A, W) - D(C, W)` inside `(x, y)`) are not generated. They are entailed only
//!   through propagation along the wait, and dropping one in step 3 leaves this dispatcher
//!   without a deadline (regression test `minimal_keeps_deadlines_behind_waits`: `X2` lost its
//!   deadline `Z + 6` behind the stand-in `X1 -> X2: 0`). Without them the output is no longer
//!   minimal in the paper's sense, only smaller (about 15 to 20 % fewer edges and waits than
//!   Morris's output on the random test networks, a quarter of the closure's).
//! - The paper's `dispstn` reorients labelled edges onto the leaders of rigid components;
//!   here the labelled edges and waits stay on their own points (the dispatcher ties members
//!   to their leader by two edges).
//! - Waits of contingent points (cross-case intermediates) take part in step 1 but are not
//!   output, so they cannot dominate in step 4.
//!
//! Dispatchability of the result is checked by simulating the dispatcher (every agent and
//! nature policy) against every requirement on random networks: evidence, not a proof.
//!
//! # Cost
//!
//! Step 1 runs Floyd-Warshall at most `k + 1` times: `O(k n^3)`. Step 2 is `O(n^3)`. Cells:
//! three `n * n` matrices (ordinary edges, stand-ins, distances).

use alloc::vec;
use alloc::vec::Vec;

use super::Stnu;
use super::estnu::{Estnu, point_kinds};
use crate::budget::Meter;
use crate::dispatch::{DispatchEdge, DispatchableNetwork, Wait};
use crate::stn::floyd_warshall::{is_negative, relax_through};
use crate::stn::{MinimalNetwork, steps};
use crate::{Budget, Error, Ticks, TimePoint};

/// A wait `waiter -> A_link: C:weight`, `weight < 0`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LabelledWait {
    /// The waiting point.
    waiter: usize,
    /// The link whose contingent point ends the wait.
    link: usize,
    /// Edge weight: minus the delay after the activation point.
    weight: Ticks,
}

/// Working state of the minimisation.
struct MinDisp<'s, 'm> {
    /// The network.
    stnu: &'s Stnu,
    /// Number of time points.
    n: usize,
    /// Row-major `n * n` ordinary edges.
    ordinary: Vec<Option<Ticks>>,
    /// Row-major `n * n` stand-in edges.
    stand_ins: Vec<Option<Ticks>>,
    /// Every wait, executable or contingent waiter.
    waits: Vec<LabelledWait>,
    /// Step meter.
    meter: &'m mut Meter,
}

/// Minimises the dispatchable ESTNU `estnu` of `stnu`.
///
/// # Errors
///
/// [`Error::MemoryBudgetExceeded`], [`Error::BudgetExhausted`], [`Error::OutOfMemory`],
/// [`Error::Overflow`], or [`Error::Internal`] if the ordinary edges are inconsistent (the
/// input was not DC).
pub(super) fn run(
    stnu: &Stnu,
    estnu: Estnu,
    budget: &Budget,
) -> Result<DispatchableNetwork, Error> {
    let n = estnu.n;
    budget.check_cells(
        n.checked_mul(n)
            .and_then(|c| c.checked_mul(3))
            .ok_or(Error::MemoryBudgetExceeded)?,
    )?;
    let mut meter = budget.meter();
    let mut m = MinDisp::new(stnu, estnu, &mut meter)?;

    m.gen_stand_ins()?;
    let dist = m.distances()?;
    let minimal = MinimalNetwork::new(n, dist.clone())?;
    let filtered = minimal.dispatchable(budget)?;

    let edges = m.drop_stand_ins(filtered.edges())?;
    let waits = m.undominated_waits(&dist)?;
    Ok(DispatchableNetwork::new(point_kinds(stnu)?, edges, waits))
}

impl<'s, 'm> MinDisp<'s, 'm> {
    /// Unpacks the ESTNU, fixing weak and misleading waits.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`].
    fn new(stnu: &'s Stnu, estnu: Estnu, meter: &'m mut Meter) -> Result<Self, Error> {
        let (n, k) = (estnu.n, estnu.k);
        let mut m = Self {
            stnu,
            n,
            ordinary: estnu.ordinary,
            stand_ins: vec![None; n * n],
            waits: Vec::new(),
            meter,
        };

        for (i, w) in estnu.waits.iter().enumerate() {
            if let Some(w) = *w {
                m.add_wait(i / k, i % k, w)?;
            }
        }

        Ok(m)
    }

    /// Adds the wait `waiter -> A_link: C:weight`: a weak one as an ordinary edge, a
    /// misleading one clamped to `-hi`.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`].
    fn add_wait(&mut self, waiter: usize, link: usize, weight: Ticks) -> Result<(), Error> {
        let l = &self.stnu.links[link];

        if weight.0 >= -l.lo.0 {
            lower(&mut self.ordinary[waiter * self.n + l.activation.0], weight);
            return Ok(());
        }

        let floor = l.hi.checked_neg().ok_or(Error::Overflow)?;
        self.waits.push(LabelledWait {
            waiter,
            link,
            weight: if weight < floor { floor } else { weight },
        });
        Ok(())
    }

    /// `genStandIns` without diamonds: the stand-ins of single labelled edges, then up to `k`
    /// rounds of the VAC rule.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::Overflow`] or [`Error::Internal`].
    fn gen_stand_ins(&mut self) -> Result<(), Error> {
        let n = self.n;

        for link in &self.stnu.links {
            let (a, c) = (link.activation.0, link.contingent.0);
            lower(&mut self.stand_ins[a * n + c], link.hi);
            lower(
                &mut self.stand_ins[c * n + a],
                link.lo.checked_neg().ok_or(Error::Overflow)?,
            );
        }

        for w in &self.waits {
            let l = &self.stnu.links[w.link];

            if w.weight.0 < -l.lo.0 {
                let neg_lo = l.lo.checked_neg().ok_or(Error::Overflow)?;
                lower(&mut self.stand_ins[w.waiter * n + l.activation.0], neg_lo);
            }
        }

        for _ in 0..self.stnu.links.len() {
            let dist = self.distances()?;

            if !self.vac_round(&dist)? {
                break;
            }
        }

        Ok(())
    }

    /// One round of the VAC rule over all waits; whether a stand-in was added.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn vac_round(&mut self, dist: &[Option<Ticks>]) -> Result<bool, Error> {
        let mut added = false;
        self.meter.spend(steps(self.waits.len())?)?;

        for i in 0..self.waits.len() {
            let w = self.waits[i];
            let link = self.stnu.links[w.link];
            let vac = link.hi.checked_add(w.weight).ok_or(Error::Overflow)?;
            added |= self.add_stand_in(dist, w.waiter, link.contingent.0, vac);
        }

        Ok(added)
    }

    /// Adds the stand-in `from -> to: weight` if it is tighter than `dist`; whether it was.
    fn add_stand_in(
        &mut self,
        dist: &[Option<Ticks>],
        from: usize,
        to: usize,
        weight: Ticks,
    ) -> bool {
        let n = self.n;

        if from == to || dist[from * n + to].is_some_and(|d| d <= weight) {
            return false;
        }

        lower(&mut self.stand_ins[from * n + to], weight);
        true
    }

    /// All-pairs distances of the ordinary edges plus stand-ins.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::Overflow`], or [`Error::Internal`] on a negative
    /// cycle (the input was DC, so its entailed edges are consistent).
    fn distances(&mut self) -> Result<Vec<Option<Ticks>>, Error> {
        let n = self.n;
        let mut dist: Vec<Option<Ticks>> = self
            .ordinary
            .iter()
            .zip(&self.stand_ins)
            .map(|(a, b)| match (*a, *b) {
                (Some(a), Some(b)) => Some(if a < b { a } else { b }),
                (a, b) => a.or(b),
            })
            .collect();

        for i in 0..n {
            lower(&mut dist[i * n + i], Ticks::ZERO);
        }

        let round = steps(n)?.saturating_mul(steps(n)?);

        for k in 0..n {
            self.meter.spend(round)?;
            relax_through(&mut dist, n, k)?;
        }

        if (0..n).any(|i| is_negative(dist[i * n + i])) {
            return Err(Error::Internal);
        }

        Ok(dist)
    }

    /// The filtered edges without the surviving stand-ins.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`].
    fn drop_stand_ins(&self, filtered: &[DispatchEdge]) -> Result<Vec<DispatchEdge>, Error> {
        let mut edges = Vec::new();

        for e in filtered {
            if self.stand_ins[e.from.0 * self.n + e.to.0] == Some(e.weight) {
                continue;
            }

            edges.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
            edges.push(*e);
        }

        Ok(edges)
    }

    /// `markWaits`: the waits of executable points that no ordinary path, upper-case edge or
    /// other wait dominates, as dispatcher waits.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::OutOfMemory`] or [`Error::Overflow`].
    fn undominated_waits(&mut self, dist: &[Option<Ticks>]) -> Result<Vec<Wait>, Error> {
        let n = self.n;
        let points = point_kinds(self.stnu)?;
        let executable = |p: usize| points[p] == crate::dispatch::PointKind::Executable;
        let kept: Vec<LabelledWait> = self
            .waits
            .iter()
            .copied()
            .filter(|w| executable(w.waiter))
            .collect();
        self.meter
            .spend(steps(kept.len())?.saturating_mul(steps(kept.len())?))?;
        let mut out = Vec::new();

        for w in &kept {
            let link = &self.stnu.links[w.link];
            let (v, a, c) = (w.waiter, link.activation.0, link.contingent.0);
            let by_path = dist[v * n + a].is_some_and(|d| d <= w.weight);
            let by_upper = is_negative(dist[v * n + c]);
            let by_wait = kept.iter().any(|u| {
                u.link == w.link
                    && u.waiter != v
                    && dist[v * n + u.waiter].is_some_and(|d| {
                        d < Ticks::ZERO && d.checked_add(u.weight).is_some_and(|s| s <= w.weight)
                    })
            });

            if by_path || by_upper || by_wait {
                continue;
            }

            out.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
            out.push(Wait {
                waiter: TimePoint(v),
                contingent: link.contingent,
                delay: w.weight.checked_neg().ok_or(Error::Overflow)?,
            });
        }

        Ok(out)
    }
}

/// Lowers `cell` to `value` if that is tighter.
fn lower(cell: &mut Option<Ticks>, value: Ticks) {
    if cell.is_none_or(|d| value < d) {
        *cell = Some(value);
    }
}
