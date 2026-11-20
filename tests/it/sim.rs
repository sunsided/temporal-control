//! Deterministic simulation of a dispatcher under different agent and nature policies.

use temporal_control::{DispatchableNetwork, Dispatcher, PointKind, Stn, Ticks, TimePoint};

/// Point capacity of simulated dispatchers.
pub const N: usize = 12;
/// Entry capacity of simulated dispatchers.
pub const E: usize = 512;

/// Small deterministic xorshift generator, so failures replay from the proptest seed.
#[derive(Debug, Clone)]
pub struct Rng(pub u64);

impl Rng {
    /// Next pseudo-random value.
    pub fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// A pseudo-random value in `lo..=hi`.
    pub fn range(&mut self, lo: i64, hi: i64) -> i64 {
        let span = u64::try_from(hi - lo + 1).unwrap();
        lo + i64::try_from(self.next() % span).unwrap()
    }

    /// A fair coin.
    pub fn coin(&mut self) -> bool {
        self.next() % 2 == 0
    }
}

/// How the agent picks among executable points.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Agent {
    /// Execute as soon as allowed, via [`Dispatcher::step`].
    Asap,
    /// Execute only when the window is about to close (or late in the run).
    Latest,
    /// Flip a coin per executable point and tick.
    Random,
}

/// How nature picks contingent durations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Nature {
    /// Always the shortest duration.
    Min,
    /// Always the longest duration.
    Max,
    /// Uniformly random.
    Random,
}

/// Tick at which the simulation starts.
const START: i64 = 0;
/// Tick after which points without a deadline are executed as soon as possible.
const FORCE_AFTER: i64 = 300;
/// Tick at which an incomplete run counts as stuck.
const END: i64 = 2000;

/// Runs one simulation and returns the time of every point.
///
/// # Panics
///
/// If the dispatcher reports any error or does not complete.
pub fn run(net: &DispatchableNetwork, agent: Agent, nature: Nature, rng: &mut Rng) -> Vec<Ticks> {
    let mut d: Dispatcher<N, E> = net.dispatcher().unwrap();
    let mut due: Vec<Option<i64>> = vec![None; net.points().len()];

    for t in START..=END {
        let now = Ticks(t);
        let observed: Vec<TimePoint> = (0..due.len())
            .filter(|&p| due[p] == Some(t))
            .map(TimePoint)
            .collect();

        if agent == Agent::Asap {
            let mut out = [TimePoint(0); N];
            let k = d.step(now, &observed, &mut out).unwrap();
            schedule_contingents(net, &out[..k], t, nature, rng, &mut due);
        } else {
            for &c in &observed {
                d.observe(c, now).unwrap();
            }

            d.check_deadlines(now).unwrap();
            let executed = act(&mut d, net, agent, now, rng);
            schedule_contingents(net, &executed, t, nature, rng, &mut due);
        }

        for &c in &observed {
            schedule_contingents(net, &[c], t, nature, rng, &mut due);
        }

        if d.is_complete() {
            return (0..due.len())
                .map(|p| d.time(TimePoint(p)).unwrap())
                .collect();
        }
    }

    panic!("dispatch did not complete: {d:?}");
}

/// Lets a non-ASAP agent execute points at `now` until it chooses to stop.
fn act(
    d: &mut Dispatcher<N, E>,
    net: &DispatchableNetwork,
    agent: Agent,
    now: Ticks,
    rng: &mut Rng,
) -> Vec<TimePoint> {
    let mut executed = Vec::new();

    loop {
        let candidates: Vec<TimePoint> = (0..net.points().len())
            .map(TimePoint)
            .filter(|&tp| d.can_execute(tp, now))
            .collect();
        let chosen = candidates.into_iter().find(|&tp| {
            let forced = d.window(tp).unwrap().1 == Some(now) || now.0 >= FORCE_AFTER;
            forced || (agent == Agent::Random && rng.coin())
        });

        let Some(tp) = chosen else {
            return executed;
        };

        d.execute(tp, now).unwrap();
        executed.push(tp);
    }
}

/// Draws the occurrence time of every contingent point activated by `activated` at `t`.
fn schedule_contingents(
    net: &DispatchableNetwork,
    activated: &[TimePoint],
    t: i64,
    nature: Nature,
    rng: &mut Rng,
    due: &mut [Option<i64>],
) {
    for (p, kind) in net.points().iter().enumerate() {
        let PointKind::Contingent { activation, lo, hi } = *kind else {
            continue;
        };

        if activated.contains(&activation) {
            let duration = match nature {
                Nature::Min => lo.0,
                Nature::Max => hi.0,
                Nature::Random => rng.range(lo.0, hi.0),
            };
            due[p] = Some(t + duration);
        }
    }
}

/// Asserts that `times` satisfies every constraint of `stn`.
pub fn assert_satisfies(stn: &Stn, times: &[Ticks]) {
    for c in stn.constraints() {
        let diff = times[c.to.0].0 - times[c.from.0].0;
        assert!(
            c.lo.is_none_or(|l| l.0 <= diff),
            "{c:?} violated by {times:?}"
        );
        assert!(
            c.hi.is_none_or(|h| diff <= h.0),
            "{c:?} violated by {times:?}"
        );
    }
}
