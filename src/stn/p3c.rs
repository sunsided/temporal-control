//! Partial path consistency with P³C (Planken, de Weerdt and van der Krogt, ICAPS 2008).
//!
//! The *constraint graph* has an undirected edge per pair of time points that share a
//! constraint. P³C makes it chordal and labels every edge of the chordal graph with its
//! minimal constraint, the same bounds the [`MinimalNetwork`](super::MinimalNetwork) has for
//! that pair, without computing the other pairs. On sparse networks the chordal graph has far
//! fewer than `n^2` edges.
//!
//! # Algorithm
//!
//! 1. **Triangulation and DPC (one pass).** Time points are eliminated one by one, always one
//!    with the fewest remaining neighbours (the minimum-degree heuristic the paper cites from
//!    Kjærulff; lowest index on ties). Eliminating `k` connects its remaining neighbours
//!    pairwise (fill edges) and tightens each pair `i, j` of them to
//!    `w(i, j) = min(w(i, j), w(i, k) + w(k, j))`. This is directional path consistency (DPC)
//!    of Dechter, Meiri and Pearl (1991, Fig. 9) along the elimination order, whose induced
//!    graph is the triangulation; P³C's Algorithm 3 runs DPC on a graph that is already chordal
//!    along a simplicial order, which the elimination order is for the filled graph.
//! 2. **Consistency.** When `k` is eliminated, each pair `i, k` with a remaining neighbour `i`
//!    has received every DPC tightening it will get; if `w(i, k) + w(k, i) < 0` the network is
//!    inconsistent. Every pair is checked once, when its first endpoint is eliminated. DMP's
//!    DPC checks the pair it just tightened, and its Theorem 5.11 (DPC detects every
//!    inconsistency) assumes non-empty input intervals; checking every pair also catches
//!    `lo > hi` and conflicting parallel constraints. Negative self-loops are rejected first.
//! 3. **Backward sweep.** In reverse elimination order, every `k` and every ordered pair `i, j`
//!    of the neighbours that remained when `k` was eliminated: `w(i, k) = min(w(i, k),
//!    w(i, j) + w(j, k))` and `w(k, j) = min(w(k, j), w(k, i) + w(i, j))` (Algorithm 3, lines
//!    3-8). By the paper's Theorem 3 every edge then carries its shortest-path distance.
//!
//! To get the minimal constraint of a pair that shares no constraint, add one without bounds
//! (`Stn::add_constraint(a, b, None, None)`): it adds the pair to the constraint graph but no
//! distance-graph edge, which is how the paper suggests making "all information of interest"
//! available.
//!
//! # Cost
//!
//! The paper's `Θ(t)` for `t` triangles of the chordal graph (`O(n w*^2)` for induced width
//! `w*`) assumes constant-time edge lookups. Here every edge weight lives in an ordered map per
//! node, and the elimination queue is an ordered set, so the cost is
//! `O((n + m + t) log n)` steps for `m` constraints, with `O(n + m + f)` memory for `f` fill
//! edges. A *step* is one triangle visit (one per sweep) or one constraint or node handled;
//! [`Budget::max_cells`] bounds the number of directed edges of the chordal graph (two per
//! undirected edge). The map nodes are allocated infallibly, like the other `O(n + m)`
//! structures (design section 6).

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec;
use alloc::vec::Vec;

use super::{Stn, TimePoint, steps};
use crate::budget::Meter;
use crate::{Budget, Error, Ticks};

/// The minimal constraints of a consistent STN on the edges of a chordal supergraph of its
/// constraint graph, as computed by [`Stn::partial_path_consistency`].
///
/// For every pair in the graph ([`Self::contains`]), [`Self::upper`] and [`Self::lower`] equal
/// the bounds of the [`MinimalNetwork`](super::MinimalNetwork); other pairs are not computed.
/// The graph holds every constrained pair (bounded or not) plus the fill edges of the
/// triangulation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChordalNetwork {
    /// Number of time points.
    n: usize,
    /// `adj[i][j]` is the tightest upper bound on `t_j - t_i`, `None` if unbounded. An entry
    /// `(i, j)` exists iff `(j, i)` does. No weight is `i64::MIN`, so every weight negates.
    adj: Vec<BTreeMap<usize, Option<Ticks>>>,
}

impl ChordalNetwork {
    /// Number of time points.
    #[must_use]
    pub fn time_point_count(&self) -> usize {
        self.n
    }

    /// Whether the minimal constraint between `a` and `b` was computed: `a == b`, or the pair
    /// is an edge of the chordal graph. False for unknown handles.
    #[must_use]
    pub fn contains(&self, a: TimePoint, b: TimePoint) -> bool {
        a.0 < self.n && b.0 < self.n && (a == b || self.adj[a.0].contains_key(&b.0))
    }

    /// Tightest upper bound on `t_to - t_from`; `None` if unbounded, or if the pair is not in
    /// the graph (see [`Self::contains`]) or a handle is unknown.
    #[must_use]
    pub fn upper(&self, from: TimePoint, to: TimePoint) -> Option<Ticks> {
        if from.0 >= self.n || to.0 >= self.n {
            return None;
        }

        if from == to {
            return Some(Ticks::ZERO);
        }

        self.adj[from.0].get(&to.0).copied().flatten()
    }

    /// Tightest lower bound on `t_to - t_from`; `None` as for [`Self::upper`].
    #[must_use]
    pub fn lower(&self, from: TimePoint, to: TimePoint) -> Option<Ticks> {
        self.upper(to, from).and_then(Ticks::checked_neg)
    }

    /// The edges of the chordal graph, each once as `(a, b)` with `a < b`, ascending.
    pub fn pairs(&self) -> impl Iterator<Item = (TimePoint, TimePoint)> + '_ {
        self.adj.iter().enumerate().flat_map(|(a, row)| {
            row.keys()
                .filter(move |&&b| a < b)
                .map(move |&b| (TimePoint(a), TimePoint(b)))
        })
    }
}

/// Runs P³C; `None` if the network is inconsistent.
///
/// # Errors
///
/// [`Error::MemoryBudgetExceeded`], [`Error::BudgetExhausted`], or [`Error::Overflow`] if an
/// intermediate path length overflows `i64` (on an inconsistent network this can happen before
/// the inconsistency is seen; the caller then lets Bellman-Ford decide).
pub(super) fn run(
    stn: &Stn,
    budget: &Budget,
    meter: &mut Meter,
) -> Result<Option<ChordalNetwork>, Error> {
    let Some(mut graph) = Graph::new(stn, budget, meter)? else {
        return Ok(None);
    };

    let Some(order) = graph.eliminate(meter)? else {
        return Ok(None);
    };

    graph.backward_sweep(&order, meter)?;
    graph.finish().map(Some)
}

/// The chordal graph being built, with its edge weights.
struct Graph<'b> {
    /// As [`ChordalNetwork::adj`], growing with fill edges.
    adj: Vec<BTreeMap<usize, Option<Ticks>>>,
    /// Directed edges so far, checked against the budget's cells.
    cells: usize,
    /// The budget, for the cell limit.
    budget: &'b Budget,
}

/// One eliminated time point and the neighbours it still had: they form a clique.
struct Eliminated {
    /// The time point.
    point: usize,
    /// Its neighbours not eliminated before it.
    later: Vec<usize>,
}

impl<'b> Graph<'b> {
    /// The constraint graph of `stn` with the tightest bound per pair; `None` if a self-loop
    /// is negative.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`] or [`Error::BudgetExhausted`].
    fn new(stn: &Stn, budget: &'b Budget, meter: &mut Meter) -> Result<Option<Self>, Error> {
        meter.spend(steps(stn.constraints.len())?.saturating_add(steps(stn.edges.len())?))?;
        let mut graph = Self {
            adj: vec![BTreeMap::new(); stn.time_points],
            cells: 0,
            budget,
        };

        for c in &stn.constraints {
            if c.from != c.to {
                graph.link(c.from.0, c.to.0)?;
            }
        }

        for e in &stn.edges {
            if e.from != e.to {
                graph.tighten(e.from, e.to, e.weight);
            } else if e.weight < Ticks::ZERO {
                return Ok(None);
            }
        }

        Ok(Some(graph))
    }

    /// Adds the undirected edge `{i, j}` with unbounded weights if it is new; returns whether
    /// it was.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`] if the graph outgrows [`Budget::max_cells`].
    fn link(&mut self, i: usize, j: usize) -> Result<bool, Error> {
        if self.adj[i].contains_key(&j) {
            return Ok(false);
        }

        self.cells = self
            .cells
            .checked_add(2)
            .ok_or(Error::MemoryBudgetExceeded)?;
        self.budget.check_cells(self.cells)?;
        self.adj[i].insert(j, None);
        self.adj[j].insert(i, None);
        Ok(true)
    }

    /// The weight of the existing edge `i -> j`; `None` if unbounded.
    fn weight(&self, i: usize, j: usize) -> Option<Ticks> {
        self.adj[i].get(&j).copied().flatten()
    }

    /// Lowers the weight of the existing edge `i -> j` to `w` if that is tighter.
    fn tighten(&mut self, i: usize, j: usize, w: Ticks) {
        if let Some(cell) = self.adj[i].get_mut(&j) {
            if cell.is_none_or(|old| w < old) {
                *cell = Some(w);
            }
        }
    }

    /// Lowers `i -> j` to the length of `i -> k -> j` if both edges are bounded and that is
    /// tighter.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if the path length overflows.
    fn relax(&mut self, i: usize, k: usize, j: usize) -> Result<(), Error> {
        if let (Some(a), Some(b)) = (self.weight(i, k), self.weight(k, j)) {
            self.tighten(i, j, a.checked_add(b).ok_or(Error::Overflow)?);
        }

        Ok(())
    }

    /// Triangulates by minimum-degree elimination while enforcing DPC along the elimination
    /// order. Returns the eliminated points in order, or `None` if the network is
    /// inconsistent.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`], [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn eliminate(&mut self, meter: &mut Meter) -> Result<Option<Vec<Eliminated>>, Error> {
        let n = self.adj.len();
        let mut degree: Vec<usize> = self.adj.iter().map(BTreeMap::len).collect();
        let mut queue: BTreeSet<(usize, usize)> = degree.iter().copied().zip(0..n).collect();
        let mut done = vec![false; n];
        let mut order = Vec::with_capacity(n);

        while let Some((_, k)) = queue.pop_first() {
            meter.spend(1)?;
            done[k] = true;
            let later: Vec<usize> = self.adj[k].keys().copied().filter(|&i| !done[i]).collect();

            for &i in &later {
                requeue(&mut queue, &mut degree, i, |d| d - 1);
            }

            if !self.eliminate_one(k, &later, &mut queue, &mut degree, meter)? {
                return Ok(None);
            }

            order.push(Eliminated { point: k, later });
        }

        Ok(Some(order))
    }

    /// Eliminates `k` with remaining neighbours `later`: checks each pair `i, k`, then fills
    /// and tightens every pair of `later`. Returns false if the network is inconsistent.
    ///
    /// # Errors
    ///
    /// [`Error::MemoryBudgetExceeded`], [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn eliminate_one(
        &mut self,
        k: usize,
        later: &[usize],
        queue: &mut BTreeSet<(usize, usize)>,
        degree: &mut [usize],
        meter: &mut Meter,
    ) -> Result<bool, Error> {
        if later.iter().any(|&i| self.is_negative_pair(i, k)) {
            return Ok(false);
        }

        for &i in later {
            meter.spend(steps(later.len())?)?;

            for &j in later.iter().filter(|&&j| j != i) {
                if self.link(i, j)? {
                    requeue(queue, degree, i, |d| d + 1);
                    requeue(queue, degree, j, |d| d + 1);
                }

                self.relax(i, k, j)?;
            }
        }

        Ok(true)
    }

    /// Whether `w(i, k) + w(k, i) < 0`, an inconsistent pair.
    fn is_negative_pair(&self, i: usize, k: usize) -> bool {
        match (self.weight(i, k), self.weight(k, i)) {
            (Some(a), Some(b)) => i128::from(a.0) + i128::from(b.0) < 0,
            _ => false,
        }
    }

    /// The backward sweep of P³C (Algorithm 3, lines 3-8), in reverse elimination order.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::Overflow`].
    fn backward_sweep(&mut self, order: &[Eliminated], meter: &mut Meter) -> Result<(), Error> {
        for Eliminated { point: k, later } in order.iter().rev() {
            for &i in later {
                meter.spend(steps(later.len())?)?;

                for &j in later.iter().filter(|&&j| j != i) {
                    self.relax(i, j, *k)?;
                    self.relax(*k, i, j)?;
                }
            }
        }

        Ok(())
    }

    /// The finished network.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a weight is `i64::MIN`, whose negation is not representable.
    fn finish(self) -> Result<ChordalNetwork, Error> {
        let rows = self.adj.iter().flat_map(BTreeMap::values);

        if rows.flatten().any(|w| w.0 == i64::MIN) {
            return Err(Error::Overflow);
        }

        Ok(ChordalNetwork {
            n: self.adj.len(),
            adj: self.adj,
        })
    }
}

/// Moves `v` in the elimination queue after changing its degree with `change`.
fn requeue(
    queue: &mut BTreeSet<(usize, usize)>,
    degree: &mut [usize],
    v: usize,
    change: impl FnOnce(usize) -> usize,
) {
    if queue.remove(&(degree[v], v)) {
        degree[v] = change(degree[v]);
        queue.insert((degree[v], v));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Verdict;

    /// An STN with `n` points and the given `(from, to, lo, hi)` constraints.
    fn stn(n: usize, constraints: &[(usize, usize, Option<i64>, Option<i64>)]) -> Stn {
        let mut stn = Stn::new();

        for _ in 0..n {
            stn.add_time_point().unwrap();
        }

        for &(from, to, lo, hi) in constraints {
            let (lo, hi) = (lo.map(Ticks), hi.map(Ticks));
            stn.add_constraint(TimePoint(from), TimePoint(to), lo, hi)
                .unwrap();
        }

        stn
    }

    /// The chordal network of `stn`; panics if inconsistent.
    fn ppc(stn: &Stn) -> ChordalNetwork {
        match stn.partial_path_consistency(&Budget::UNLIMITED).unwrap() {
            Verdict::Consistent(net) => net,
            Verdict::Inconsistent(_) => panic!("expected consistent"),
        }
    }

    /// Whether `stn` is inconsistent according to P³C.
    fn inconsistent(stn: &Stn) -> bool {
        let verdict = stn.partial_path_consistency(&Budget::UNLIMITED).unwrap();
        matches!(verdict, Verdict::Inconsistent(_))
    }

    /// A pair of interest without bounds gets the implied interval of a chain.
    #[test]
    fn unbounded_pair_of_interest_is_computed() {
        let net = ppc(&stn(
            3,
            &[
                (0, 1, Some(2), Some(5)),
                (1, 2, Some(2), Some(5)),
                (0, 2, None, None),
            ],
        ));

        assert_eq!(net.upper(TimePoint(0), TimePoint(2)), Some(Ticks(10)));
        assert_eq!(net.lower(TimePoint(0), TimePoint(2)), Some(Ticks(4)));
    }

    /// A 4-cycle is not chordal: it gets one fill edge, and every edge, the fill edge
    /// included, carries the minimal constraint.
    #[test]
    fn four_cycle_gets_a_minimal_fill_edge() {
        // 0 -> 1 -> 2 -> 3 each [1, 2] and 0 -> 3 in [0, 4], which tightens to [3, 4].
        let stn = stn(
            4,
            &[
                (0, 1, Some(1), Some(2)),
                (1, 2, Some(1), Some(2)),
                (2, 3, Some(1), Some(2)),
                (0, 3, Some(0), Some(4)),
            ],
        );
        let net = ppc(&stn);
        let Verdict::Consistent(min) = stn.minimal_network(&Budget::UNLIMITED).unwrap() else {
            panic!("consistent");
        };

        assert_eq!(net.pairs().count(), 5);

        for (a, b) in net.pairs() {
            assert_eq!(net.upper(a, b), min.upper(a, b), "{a:?} -> {b:?}");
            assert_eq!(net.upper(b, a), min.upper(b, a), "{b:?} -> {a:?}");
        }

        assert_eq!(net.lower(TimePoint(0), TimePoint(3)), Some(Ticks(3)));
        assert_eq!(net.upper(TimePoint(1), TimePoint(2)), Some(Ticks(2)));
    }

    /// Contradictions on a single pair, which DPC's triangle updates never touch.
    #[test]
    fn single_pair_contradictions_are_found() {
        assert!(inconsistent(&stn(2, &[(0, 1, Some(5), Some(3))])));
        assert!(inconsistent(&stn(
            2,
            &[(0, 1, Some(5), None), (0, 1, None, Some(3))]
        )));
        assert!(inconsistent(&stn(1, &[(0, 0, Some(1), None)])));
        assert!(!inconsistent(&stn(1, &[(0, 0, Some(0), Some(0))])));
    }

    /// A negative cycle over three points is found by the DPC pass.
    #[test]
    fn triangle_negative_cycle_is_found() {
        let stn = stn(
            3,
            &[
                (0, 1, Some(2), None),
                (1, 2, Some(2), None),
                (0, 2, None, Some(3)),
            ],
        );
        assert!(inconsistent(&stn));
    }

    /// The pathological instances `P_t` of the paper (Definition 2), on which `ΔSTP` processes
    /// `Ω(t^2)` triangles: a zero-weight cycle filled with chords whose weights make longer
    /// paths shorter. Every minimal weight is zero.
    #[test]
    fn pathological_instances_have_zero_minimal_weights() {
        for t in 1..=8usize {
            let mut constraints = Vec::new();
            let w = |x: usize| Some(i64::try_from(x).unwrap());

            for i in 0..t + 2 {
                constraints.push((i, (i + 1) % (t + 2), None, Some(0)));
            }

            for i in 1..=t {
                for j in i + 2..=t + 1 {
                    if matches!((i + j).checked_sub(t), Some(1 | 2)) {
                        constraints.push((i, j, None, w(j - i - 1)));
                        constraints.push((j, i, None, w(t - (j - i - 1))));
                    }
                }
            }

            let net = ppc(&stn(t + 2, &constraints));

            for (a, b) in net.pairs() {
                assert_eq!(
                    net.upper(a, b),
                    Some(Ticks::ZERO),
                    "t = {t}, {a:?} -> {b:?}"
                );
                assert_eq!(
                    net.upper(b, a),
                    Some(Ticks::ZERO),
                    "t = {t}, {b:?} -> {a:?}"
                );
            }
        }
    }

    /// The cell budget bounds the chordal graph, fill edges included.
    #[test]
    fn cell_budget_is_enforced() {
        let stn = stn(
            4,
            &[
                (0, 1, None, Some(1)),
                (1, 2, None, Some(1)),
                (2, 3, None, Some(1)),
                (3, 0, None, Some(1)),
            ],
        );
        let tight = Budget {
            max_steps: u64::MAX,
            max_cells: 8,
        };
        let enough = Budget {
            max_steps: u64::MAX,
            max_cells: 10,
        };

        assert_eq!(
            stn.partial_path_consistency(&tight),
            Err(Error::MemoryBudgetExceeded)
        );
        assert!(stn.partial_path_consistency(&enough).is_ok());
    }
}
