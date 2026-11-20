//! Brute-force dynamic-controllability oracle: solves the integer-time execution game.
//!
//! Time point 0 is executed at tick 0. At every tick `t`, nature chooses which activated
//! contingent points happen at `t` (any whose window `[a + lo, a + hi]` contains `t`; those with
//! `t = a + hi` must), and the agent chooses which unexecuted executable points to execute at
//! `t`. With [`Reaction::Instant`] nature moves first and the agent sees its choice; with
//! [`Reaction::NextTick`] the agent moves first. The agent wins if every constraint holds once
//! all points happened. Exponential; only for tiny networks with bounded horizons.

use std::collections::HashMap;

use crate::stnu_gen::StnuSpec;

/// Who moves first within a tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reaction {
    /// Nature first: the agent may react to an observation in the same tick.
    Instant,
    /// Agent first: observations at `t` can only influence decisions at `t + 1`.
    NextTick,
}

/// Game state key: current tick and the time of every point.
type Key = (i64, Vec<Option<i64>>);

/// Solver for one network.
struct Game<'a> {
    /// The network.
    spec: &'a StnuSpec,
    /// Move order within a tick.
    reaction: Reaction,
    /// Last tick at which anything may still happen.
    last_tick: i64,
    /// Memoized outcomes.
    memo: HashMap<Key, bool>,
}

/// Whether the agent has a dynamic strategy, by exhaustive game search.
pub fn is_dc(spec: &StnuSpec, reaction: Reaction) -> bool {
    let n = spec.point_count();
    let mut times = vec![None; n];
    times[0] = Some(0);
    let mut game = Game {
        spec,
        reaction,
        last_tick: spec.horizon + spec.links.iter().map(|l| l.2).sum::<i64>() + 1,
        memo: HashMap::new(),
    };
    !game.lost(-1, &times) && game.wins(0, times)
}

impl Game<'_> {
    /// Value of the position at the start of tick `t`.
    fn wins(&mut self, t: i64, times: Vec<Option<i64>>) -> bool {
        if times.iter().all(Option::is_some) {
            return true;
        }

        if t > self.last_tick {
            return false;
        }

        let key = (t, times);

        if let Some(&v) = self.memo.get(&key) {
            return v;
        }

        let (t, times) = key;
        let value = self.solve_tick(t, &times);
        self.memo.insert((t, times), value);
        value
    }

    /// Enumerates the moves of tick `t` in the configured order.
    fn solve_tick(&mut self, t: i64, times: &[Option<i64>]) -> bool {
        let (optional, forced) = self.nature_options(t, times);
        let agent = self.agent_options(times);
        let natures = subsets(&optional);

        match self.reaction {
            Reaction::Instant => natures.iter().all(|n| {
                subsets(&agent)
                    .iter()
                    .any(|a| self.play(t, times, &forced, n, a))
            }),
            Reaction::NextTick => subsets(&agent)
                .iter()
                .any(|a| natures.iter().all(|n| self.play(t, times, &forced, n, a))),
        }
    }

    /// Applies both moves at tick `t` and continues.
    fn play(
        &mut self,
        t: i64,
        times: &[Option<i64>],
        forced: &[usize],
        nature: &[usize],
        agent: &[usize],
    ) -> bool {
        let mut next = times.to_vec();

        for &p in forced.iter().chain(nature).chain(agent) {
            next[p] = Some(t);
        }

        self.after_moves(t, next)
    }

    /// Prunes positions that already lost, then moves to tick `t + 1`.
    fn after_moves(&mut self, t: i64, times: Vec<Option<i64>>) -> bool {
        if self.lost(t, &times) {
            return false;
        }

        self.wins(t + 1, times)
    }

    /// Contingent points nature may (first) or must (second) execute at `t`.
    fn nature_options(&self, t: i64, times: &[Option<i64>]) -> (Vec<usize>, Vec<usize>) {
        let mut optional = Vec::new();
        let mut forced = Vec::new();

        for (i, &(act, lo, hi)) in self.spec.links.iter().enumerate() {
            let c = self.spec.contingent(i);

            let (Some(a), None) = (times[act], times[c]) else {
                continue;
            };

            if t == a + hi {
                forced.push(c);
            } else if a + lo <= t && t < a + hi {
                optional.push(c);
            }
        }

        (optional, forced)
    }

    /// Unexecuted executable points.
    fn agent_options(&self, times: &[Option<i64>]) -> Vec<usize> {
        (0..self.spec.n_exec)
            .filter(|&p| times[p].is_none())
            .collect()
    }

    /// Whether a constraint is already violated, or can no longer be met, after tick `t`.
    fn lost(&self, t: i64, times: &[Option<i64>]) -> bool {
        self.spec
            .edges()
            .any(|(u, v, w)| match (times[u], times[v]) {
                (Some(tu), Some(tv)) => tv - tu > w,
                (Some(tu), None) => tu + w <= t,
                _ => false,
            })
    }
}

/// All subsets of `items`.
fn subsets(items: &[usize]) -> Vec<Vec<usize>> {
    (0..1u32 << items.len())
        .map(|mask| {
            items
                .iter()
                .enumerate()
                .filter(|(i, _)| mask & (1 << i) != 0)
                .map(|(_, &p)| p)
                .collect()
        })
        .collect()
}
