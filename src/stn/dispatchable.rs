//! Minimal dispatchable STNs by dominated-edge filtering (Muscettola, Morris and Tsamardinos 1998).
//!
//! The minimal network (all-pairs shortest paths) is dispatchable but dense. Filtering keeps a
//! subset of its edges that is still dispatchable:
//!
//! 1. **Rigid components.** Points `i, j` with `D[i][j] + D[j][i] = 0` have a fixed offset.
//!    Each component gets a leader, its earliest member (lowest index on ties). Every other
//!    member is tied to the leader by the two edges `leader -> m` and `m -> leader`, and all
//!    other constraints are expressed between leaders only. With the earliest member as leader,
//!    members never need to execute before their leader, so the dispatcher learns their exact
//!    time from the leader in time.
//! 2. **Dominance among leaders.** For leaders `a, b, c` (pairwise distinct):
//!    - a non-negative edge `a -> c` is *upper-dominated* by a non-negative edge `b -> c` if
//!      `D[a][b] + D[b][c] = D[a][c]`;
//!    - a negative edge `a -> c` is *lower-dominated* by a negative edge `a -> b` if
//!      `D[a][b] + D[b][c] = D[a][c]`.
//!
//!    Dominated edges are dropped. Leaders are pairwise non-rigid, so no two edges dominate
//!    each other and all dominated edges can be dropped at once.
//!
//! Cost: `O(n^3)` steps, `O(n)` extra memory plus the output.

use alloc::vec::Vec;

use super::{MinimalNetwork, TimePoint, steps};
use crate::budget::Meter;
use crate::dispatch::{DispatchEdge, DispatchableNetwork, PointKind};
use crate::{Budget, Error, Ticks};

impl MinimalNetwork {
    /// Derives the minimal dispatchable network by dominated-edge filtering.
    ///
    /// The result is equivalent to this network (same all-pairs distances) and dispatchable by
    /// a [`Dispatcher`](crate::Dispatcher).
    ///
    /// # Errors
    ///
    /// - [`Error::BudgetExhausted`] if the `O(n^3)` filtering does not fit the step budget.
    /// - [`Error::OutOfMemory`] if the output cannot be allocated.
    /// - [`Error::Overflow`] if a distance sum overflows.
    pub fn dispatchable(&self, budget: &Budget) -> Result<DispatchableNetwork, Error> {
        let mut meter = budget.meter();
        let leaders = self.rigid_leaders(&mut meter)?;
        let mut edges = Vec::new();

        self.push_rigid_edges(&leaders, &mut edges)?;
        self.push_undominated_edges(&leaders, &mut meter, &mut edges)?;

        let mut points = Vec::new();
        points
            .try_reserve_exact(self.n)
            .map_err(|_| Error::OutOfMemory)?;
        points.resize(self.n, PointKind::Executable);

        Ok(DispatchableNetwork::new(points, edges, Vec::new()))
    }

    /// Distance `i -> j` by index.
    fn d(&self, i: usize, j: usize) -> Option<Ticks> {
        self.dist[i * self.n + j]
    }

    /// Whether `i` and `j` have a fixed offset.
    fn rigid(&self, i: usize, j: usize) -> bool {
        matches!((self.d(i, j), self.d(j, i)), (Some(a), Some(b)) if a.0.checked_add(b.0) == Some(0))
    }

    /// Leader of each point's rigid component: its earliest member, lowest index on ties.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::OutOfMemory`].
    fn rigid_leaders(&self, meter: &mut Meter) -> Result<Vec<usize>, Error> {
        let n = self.n;
        let mut leaders = Vec::new();
        leaders
            .try_reserve_exact(n)
            .map_err(|_| Error::OutOfMemory)?;

        for i in 0..n {
            meter.spend(steps(n)?)?;
            let leader = (0..n)
                .filter(|&j| self.rigid(i, j))
                .min_by_key(|&j| (self.d(i, j).map(|o| o.0), j))
                .unwrap_or(i);
            leaders.push(leader);
        }

        Ok(leaders)
    }

    /// Ties every non-leader to its leader with the two rigid edges.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`].
    fn push_rigid_edges(
        &self,
        leaders: &[usize],
        edges: &mut Vec<DispatchEdge>,
    ) -> Result<(), Error> {
        for (m, &l) in leaders.iter().enumerate() {
            if m == l {
                continue;
            }

            for (from, to) in [(l, m), (m, l)] {
                if let Some(weight) = self.d(from, to) {
                    push_edge(edges, from, to, weight)?;
                }
            }
        }

        Ok(())
    }

    /// Adds every finite edge between distinct leaders that no third leader dominates.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`], [`Error::Overflow`] or [`Error::OutOfMemory`].
    fn push_undominated_edges(
        &self,
        leaders: &[usize],
        meter: &mut Meter,
        edges: &mut Vec<DispatchEdge>,
    ) -> Result<(), Error> {
        let mut heads = Vec::new();
        heads
            .try_reserve_exact(self.n)
            .map_err(|_| Error::OutOfMemory)?;
        heads.extend((0..self.n).filter(|&i| leaders[i] == i));

        for &a in &heads {
            meter.spend(steps(heads.len())?.saturating_mul(steps(heads.len())?))?;

            for &c in &heads {
                let Some(w) = self.d(a, c) else {
                    continue;
                };

                if a != c && !self.dominated(&heads, a, c, w)? {
                    push_edge(edges, a, c, w)?;
                }
            }
        }

        Ok(())
    }

    /// Whether edge `a -> c` with weight `w` is dominated through some other leader.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a distance sum overflows.
    fn dominated(&self, heads: &[usize], a: usize, c: usize, w: Ticks) -> Result<bool, Error> {
        for &b in heads {
            if b == a || b == c {
                continue;
            }

            let (Some(ab), Some(bc)) = (self.d(a, b), self.d(b, c)) else {
                continue;
            };

            let through_b = ab.checked_add(bc).ok_or(Error::Overflow)? == w;
            let dominating = if w >= Ticks::ZERO { bc } else { ab };
            let same_sign = (dominating >= Ticks::ZERO) == (w >= Ticks::ZERO);

            if through_b && same_sign {
                return Ok(true);
            }
        }

        Ok(false)
    }
}

/// Appends an edge with a fallible allocation.
///
/// # Errors
///
/// [`Error::OutOfMemory`].
fn push_edge(
    edges: &mut Vec<DispatchEdge>,
    from: usize,
    to: usize,
    weight: Ticks,
) -> Result<(), Error> {
    edges.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
    edges.push(DispatchEdge {
        from: TimePoint(from),
        to: TimePoint(to),
        weight,
    });
    Ok(())
}
