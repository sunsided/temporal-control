//! Dynamic controllability by closure of the labelled distance graph.
//!
//! The labelled distance graph of an STNU has, besides the ordinary edges of the requirements,
//! per contingent link `(A, l, u, C)` the ordinary edges `A -> C: u` and `C -> A: -l`, the
//! lower-case edge `A -> C: c:l` and the upper-case edge `C -> A: C:-u`. Upper-case edges always
//! point to the activation point of their label, so they fit an `n * k` matrix `W`, where
//! `W[x][c]` is the weight of the upper-case edge `x -> A_c` labelled `c`. Ordinary edges form
//! the `n * n` matrix `D`. Lower-case edges are fixed and read from the links.
//!
//! One round applies, in order:
//!
//! 1. **No-case**: Floyd-Warshall on `D`. A negative diagonal entry means not DC.
//! 2. **Upper-case**: `W[x][c] = min_y D[x][y] + W[y][c]` (one pass suffices on a closed `D`).
//! 3. **Cross-case**: `A_d -> C_d: d:l_d` and `C_d -> A_c: c:w` with `c != d`, `w < 0` give
//!    `W[A_d][c] <= l_d + w`.
//! 4. **Lower-case**: `A_d -> C_d: d:l_d` and `C_d -> y: v` with `v < 0` give
//!    `D[A_d][y] <= l_d + v`.
//! 5. **Label removal**: `W[x][c] >= -l_c` gives `D[x][A_c] <= W[x][c]`.
//! 6. **All-max projection**: `D` plus `W` read as ordinary edges must have no negative cycle.
//!
//! Rounds repeat until nothing changes. Every derived ordinary edge holds in every execution of
//! every valid dynamic strategy, so for a DC network the values are bounded below and the
//! closure terminates; for a non-DC network a semi-reducible negative cycle reduces to a
//! negative cycle that step 1 or 6 finds.

use alloc::vec::Vec;

use super::{Controllability, Stnu};
use crate::budget::Meter;
use crate::dispatch::{DispatchEdge, DispatchableNetwork, PointKind, Wait};
use crate::stn::floyd_warshall::{is_negative, relax_through};
use crate::stn::steps;
use crate::{Budget, Error, Ticks, TimePoint};

/// The labelled distance matrices during the closure.
struct Labelled {
    /// Number of time points.
    n: usize,
    /// Number of contingent links.
    k: usize,
    /// Row-major `n * n` ordinary distances; `None` is `+infinity`.
    d: Vec<Option<Ticks>>,
    /// Row-major `n * k` upper-case distances `x -> A_c` labelled `c`.
    w: Vec<Option<Ticks>>,
    /// Scratch `n * n` matrix for the all-max projection.
    scratch: Vec<Option<Ticks>>,
}

/// Runs the closure and builds the extended dispatchable network.
///
/// # Errors
///
/// See [`Stnu::dispatchable`].
pub(super) fn run(
    stnu: &Stnu,
    budget: &Budget,
) -> Result<Controllability<DispatchableNetwork>, Error> {
    let mut meter = budget.meter();
    let mut g = Labelled::new(stnu, budget, &mut meter)?;

    loop {
        let Some(changed) = g.round(stnu, &mut meter)? else {
            return Ok(Controllability::Uncontrollable);
        };

        if !changed {
            return g.into_network(stnu).map(Controllability::Controllable);
        }
    }
}

impl Labelled {
    /// Allocates the matrices and loads the initial edges.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`], [`Error::OutOfMemory`], [`Error::BudgetExhausted`] or
    /// [`Error::Overflow`].
    fn new(stnu: &Stnu, budget: &Budget, meter: &mut Meter) -> Result<Self, Error> {
        let n = stnu.time_point_count();
        let k = stnu.links.len();
        let nn = n.checked_mul(n).ok_or(Error::MemoryBudgetExceeded)?;
        let nk = n.checked_mul(k).ok_or(Error::MemoryBudgetExceeded)?;
        let cells = nn
            .checked_mul(2)
            .and_then(|c| c.checked_add(nk))
            .ok_or(Error::MemoryBudgetExceeded)?;
        budget.check_cells(cells)?;
        meter.spend(steps(cells)?)?;

        let mut g = Self {
            n,
            k,
            d: matrix(nn)?,
            w: matrix(nk)?,
            scratch: matrix(nn)?,
        };

        g.load(stnu)?;
        Ok(g)
    }

    /// Loads requirement edges, contingent ordinary edges and original upper-case edges.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a lower bound cannot be negated.
    fn load(&mut self, stnu: &Stnu) -> Result<(), Error> {
        let n = self.n;

        for i in 0..n {
            self.d[i * n + i] = Some(Ticks::ZERO);
        }

        for c in stnu.requirements.constraints() {
            let (u, v) = (c.from.0, c.to.0);

            if let Some(hi) = c.hi {
                lower(&mut self.d[u * n + v], hi);
            }

            if let Some(lo) = c.lo {
                lower(
                    &mut self.d[v * n + u],
                    lo.checked_neg().ok_or(Error::Overflow)?,
                );
            }
        }

        for (c, link) in stnu.links.iter().enumerate() {
            let (a, x) = (link.activation.0, link.contingent.0);
            let neg_lo = link.lo.checked_neg().ok_or(Error::Overflow)?;
            let neg_hi = link.hi.checked_neg().ok_or(Error::Overflow)?;
            lower(&mut self.d[a * n + x], link.hi);
            lower(&mut self.d[x * n + a], neg_lo);
            lower(&mut self.w[x * self.k + c], neg_hi);
        }

        Ok(())
    }

    /// Runs one round of all rules.
    ///
    /// Returns `None` if a negative cycle shows the network is not DC, else whether anything
    /// changed.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn round(&mut self, stnu: &Stnu, meter: &mut Meter) -> Result<Option<bool>, Error> {
        let before_d = self.d.clone();
        let before_w = self.w.clone();

        if !self.close_ordinary(meter)? {
            return Ok(None);
        }

        self.upper_case(meter)?;
        self.cross_case(stnu)?;
        self.lower_case(stnu, meter)?;
        self.label_removal(stnu);

        if !self.all_max_consistent(stnu, meter)? {
            return Ok(None);
        }

        Ok(Some(self.d != before_d || self.w != before_w))
    }

    /// Floyd-Warshall on `D`; returns false on a negative diagonal entry.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn close_ordinary(&mut self, meter: &mut Meter) -> Result<bool, Error> {
        close(&mut self.d, self.n, meter)
    }

    /// Upper-case rule over the closed `D`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn upper_case(&mut self, meter: &mut Meter) -> Result<(), Error> {
        let (n, k) = (self.n, self.k);
        meter.spend(
            steps(n)?
                .saturating_mul(steps(n)?)
                .saturating_mul(steps(k)?),
        )?;

        for c in 0..k {
            for x in 0..n {
                let mut best = self.w[x * k + c];

                for y in 0..n {
                    if let (Some(dxy), Some(wyc)) = (self.d[x * n + y], self.w[y * k + c]) {
                        let candidate = dxy.checked_add(wyc).ok_or(Error::Overflow)?;
                        lower(&mut best, candidate);
                    }
                }

                self.w[x * k + c] = best;
            }
        }

        Ok(())
    }

    /// Cross-case rule for every pair of distinct links.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`].
    fn cross_case(&mut self, stnu: &Stnu) -> Result<(), Error> {
        let k = self.k;

        for (d, link) in stnu.links.iter().enumerate() {
            for c in (0..k).filter(|&c| c != d) {
                let Some(w) = self.w[link.contingent.0 * k + c] else {
                    continue;
                };

                if w < Ticks::ZERO {
                    let candidate = link.lo.checked_add(w).ok_or(Error::Overflow)?;
                    lower(&mut self.w[link.activation.0 * k + c], candidate);
                }
            }
        }

        Ok(())
    }

    /// Lower-case rule for every link and every target.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn lower_case(&mut self, stnu: &Stnu, meter: &mut Meter) -> Result<(), Error> {
        let n = self.n;
        meter.spend(steps(n)?.saturating_mul(steps(self.k)?))?;

        for link in &stnu.links {
            let (a, x) = (link.activation.0, link.contingent.0);

            for y in 0..n {
                let Some(v) = self.d[x * n + y] else {
                    continue;
                };

                if v < Ticks::ZERO {
                    let candidate = link.lo.checked_add(v).ok_or(Error::Overflow)?;
                    lower(&mut self.d[a * n + y], candidate);
                }
            }
        }

        Ok(())
    }

    /// Label removal: upper-case edges no tighter than `-l_c` become ordinary.
    fn label_removal(&mut self, stnu: &Stnu) {
        let (n, k) = (self.n, self.k);

        for (c, link) in stnu.links.iter().enumerate() {
            let a = link.activation.0;

            for x in 0..n {
                let Some(w) = self.w[x * k + c] else {
                    continue;
                };

                if w.0 >= -link.lo.0 {
                    lower(&mut self.d[x * n + a], w);
                }
            }
        }
    }

    /// Whether `D` plus `W` read as ordinary edges is free of negative cycles.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn all_max_consistent(&mut self, stnu: &Stnu, meter: &mut Meter) -> Result<bool, Error> {
        let (n, k) = (self.n, self.k);
        self.scratch.copy_from_slice(&self.d);

        for (c, link) in stnu.links.iter().enumerate() {
            let a = link.activation.0;

            for x in 0..n {
                if let Some(w) = self.w[x * k + c] {
                    lower(&mut self.scratch[x * n + a], w);
                }
            }
        }

        close(&mut self.scratch, n, meter)
    }

    /// Builds the extended dispatchable network from the closed matrices.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`], or [`Error::Overflow`] if a wait cannot be negated.
    fn into_network(self, stnu: &Stnu) -> Result<DispatchableNetwork, Error> {
        let (n, k) = (self.n, self.k);
        let mut points = Vec::new();
        points
            .try_reserve_exact(n)
            .map_err(|_| Error::OutOfMemory)?;
        points.resize(n, PointKind::Executable);

        for link in &stnu.links {
            points[link.contingent.0] = PointKind::Contingent {
                activation: link.activation,
                lo: link.lo,
                hi: link.hi,
            };
        }

        let mut edges = Vec::new();

        for x in 0..n {
            for y in (0..n).filter(|&y| y != x) {
                if let Some(weight) = self.d[x * n + y] {
                    edges.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
                    edges.push(DispatchEdge {
                        from: TimePoint(x),
                        to: TimePoint(y),
                        weight,
                    });
                }
            }
        }

        let mut waits = Vec::new();

        for (c, link) in stnu.links.iter().enumerate() {
            let a = link.activation.0;

            for x in (0..n).filter(|&x| points[x] == PointKind::Executable) {
                let Some(w) = self.w[x * k + c] else {
                    continue;
                };

                if self.d[x * n + a].is_none_or(|d| w < d) {
                    waits.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
                    waits.push(Wait {
                        waiter: TimePoint(x),
                        contingent: link.contingent,
                        delay: w.checked_neg().ok_or(Error::Overflow)?,
                    });
                }
            }
        }

        Ok(DispatchableNetwork::new(points, edges, waits))
    }
}

/// Allocates a matrix of `cells` infinite entries.
///
/// # Errors
///
/// [`Error::OutOfMemory`].
fn matrix(cells: usize) -> Result<Vec<Option<Ticks>>, Error> {
    let mut m = Vec::new();
    m.try_reserve_exact(cells).map_err(|_| Error::OutOfMemory)?;
    m.resize(cells, None);
    Ok(m)
}

/// Lowers `cell` to `value` if that is tighter.
fn lower(cell: &mut Option<Ticks>, value: Ticks) {
    if cell.is_none_or(|d| value < d) {
        *cell = Some(value);
    }
}

/// Floyd-Warshall on an `n * n` matrix; returns false on a negative diagonal entry.
///
/// # Errors
///
/// [`Error::BudgetExhausted`] or [`Error::Overflow`].
fn close(m: &mut Vec<Option<Ticks>>, n: usize, meter: &mut Meter) -> Result<bool, Error> {
    let round = steps(n)?.saturating_mul(steps(n)?);

    for k in 0..n {
        if is_negative(m[k * n + k]) {
            return Ok(false);
        }

        meter.spend(round)?;
        relax_through(m, n, k)?;
    }

    Ok(!(0..n).any(|i| is_negative(m[i * n + i])))
}
