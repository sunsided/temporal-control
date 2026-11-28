//! Multi-agent STNs: temporal decoupling, distributed consistency checking and re-decoupling.
//!
//! Several agents (rovers, a lander, an orbiter) share one plan, but they can talk to each
//! other only now and then. Each time point other than the common reference belongs to exactly
//! one agent. A constraint is *shared* when its endpoints belong to different agents; every
//! other constraint is local to one agent.
//!
//! # Temporal decoupling
//!
//! [`MultiAgentStn::decouple`] gives every point of a shared constraint a bound relative to
//! the reference point `z`, its *cut*, so that the cuts on both sides imply the shared
//! constraint. For a shared distance-graph edge `u -> v` with weight `w` (`t_v - t_u <= w`), it
//! picks a value `c` and adds `t_u - z >= c` to the owner of `u` and `t_v - z <= c + w` to the
//! owner of `v`. Each agent then dispatches its own [`Agent::local_network`] alone: every
//! combination of locally consistent schedules is consistent with the whole plan. This is the
//! temporal decoupling problem of Hunsberger (AAAI 2002); agents holding subnetworks joined by
//! shared points are also the setting of Boerkoel and Durfee (JAIR 2013).
//!
//! The cuts are chosen greedily, one shared edge at a time in constraint order, on the
//! incrementally maintained minimal network. `c` is the midpoint between
//! `max(lower(u), lower(v) - w)` and `min(upper(u), upper(v) - w)`, so the slack the edge
//! still has is split evenly between both sides. Each choice keeps the network consistent, so
//! decoupling succeeds whenever the plan is consistent. It is a heuristic: it does not
//! maximise the total flexibility left to the agents (an optimal decoupling needs a linear
//! program).
//!
//! # Distributed checking and re-decoupling
//!
//! Each [`Agent`] keeps its local constraints and its cuts. A change inside its cuts it
//! accepts alone ([`Agent::add_constraint`]); a change beyond them it records as a proposal
//! ([`Agent::propose`]) for the next communication window. When agents can communicate, each
//! one sends a [`Summary`]: its minimal network restricted to the reference and the points of
//! its shared constraints with the other agents of the group. Because the minimal network of
//! an STN is decomposable, these summaries plus the shared constraints are consistent iff the
//! agents' combined networks are ([`AgentGroup::check_consistency`]), and they have the same
//! minimal bounds on the shared points. No agent needs another's private points, and no
//! central solver is involved: every member builds the same [`AgentGroup`] from the same
//! summaries.
//!
//! The summaries include the proposals, so this check tells the group whether they fit the
//! plan. [`Agent::redecouple`] then adopts the proposals (or drops them if they do not fit)
//! and recomputes the cuts of the group's shared constraints from that [`AgentGroup`],
//! keeping the cuts towards agents outside the group. The new cuts are a pure
//! function of the summaries, so every member derives the same ones; with all agents in the
//! group they equal those of [`MultiAgentStn::decouple`] on the current constraints.
//!
//! ```mermaid
//! graph LR
//!     plan["MultiAgentStn (ground)"] -->|"decouple"| agents["Agent per rover"]
//!     agents -->|"local_network"| local["Stn: dispatch alone"]
//!     agents -->|"add_constraint (local check), propose"| agents
//!     agents -->|"summary (in a window)"| msg["Summary"]
//!     msg -->|"AgentGroup::new"| group["AgentGroup"]
//!     group -->|"check_consistency"| verdict["GroupVerdict"]
//!     group -->|"Agent::redecouple"| agents
//! ```
//!
//! **Protocol requirement.** Re-decoupling is safe only if every member of the group adopts
//! its new cuts, and none keeps the old ones. Exchange the summaries, make sure every member
//! holds the full set (for example with an acknowledgement round), and only then call
//! [`Agent::redecouple`] on each member. A member that missed the exchange keeps its old cuts,
//! and then every other member must keep theirs too.
//!
//! # Communication windows
//!
//! A relay pass or another communication opportunity whose time is not known in advance is a
//! contingent link in an [`Stnu`](crate::Stnu): it opens between `lo` and `hi` ticks after its
//! activation, and the agent learns the actual time only when it happens. Dynamic
//! controllability then guarantees that the plan works however the window falls:
//!
//! ```
//! use temporal_control::{Budget, Controllability, Stnu, Ticks};
//!
//! // The pass opens 30..50 ticks after the epoch and lasts 10 ticks.
//! let mut stnu = Stnu::new();
//! let epoch = stnu.add_time_point().expect("fits");
//! let open = stnu.add_contingent_link(epoch, Ticks(30), Ticks(50)).expect("valid");
//! let send = stnu.add_time_point().expect("fits");
//! stnu.add_constraint(open, send, Some(Ticks(0)), Some(Ticks(10))).expect("exists");
//!
//! // Sending during the pass, whenever it opens, is controllable.
//! let verdict = stnu.check_controllability(&Budget::UNLIMITED).expect("within budget");
//! assert_eq!(verdict, Controllability::Controllable(()));
//!
//! // A send deadline before the latest opening plus the pass length is not.
//! stnu.add_constraint(epoch, send, None, Some(Ticks(45))).expect("exists");
//! let verdict = stnu.check_controllability(&Budget::UNLIMITED).expect("within budget");
//! assert!(matches!(verdict, Controllability::Uncontrollable(_)));
//! ```
//!
//! Decoupling and the group operations work on STNs; uncertain durations inside one agent's
//! plan are checked on that agent's own STNU.

mod agent;
mod cuts;
mod group;

use alloc::vec::Vec;

pub use agent::Agent;
pub use group::{AgentGroup, GroupConflict, GroupVerdict, Summary};

use crate::stn::{Bound, Constraint, ConstraintId, Stn, Verdict};
use crate::{Budget, Error, Ticks, TimePoint};

/// The reference point `z` shared by all agents: index zero in every network of this module.
const REFERENCE: TimePoint = TimePoint(0);

/// Handle of an agent inside one [`MultiAgentStn`], from zero to the agent count.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AgentId(pub usize);

/// A constraint between points of two different agents.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SharedConstraint {
    /// Its handle in the [`MultiAgentStn`] that defined it.
    pub id: ConstraintId,
    /// The constraint, on the time points of the [`MultiAgentStn`].
    pub constraint: Constraint,
    /// Owner of [`Constraint::from`].
    pub from_agent: AgentId,
    /// Owner of [`Constraint::to`]. Never equal to `from_agent`.
    pub to_agent: AgentId,
}

impl SharedConstraint {
    /// The agent on the other side of this constraint from `agent`, or `None` if `agent` is on
    /// neither side.
    #[must_use]
    pub fn peer(&self, agent: AgentId) -> Option<AgentId> {
        if agent == self.from_agent {
            Some(self.to_agent)
        } else if agent == self.to_agent {
            Some(self.from_agent)
        } else {
            None
        }
    }
}

/// One bound a decoupling puts on a point of a shared constraint, relative to the reference.
///
/// [`Bound::Lower`] means `t_point - z >= value`, [`Bound::Upper`] means `t_point - z <=
/// value`. Every shared distance-graph edge `u -> v` with weight `w` yields one lower cut on
/// `u` and one upper cut on `v` whose values differ by exactly `w`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Cut {
    /// The shared constraint this cut helps imply.
    pub shared: ConstraintId,
    /// The bounded point, a point of the [`MultiAgentStn`].
    pub point: TimePoint,
    /// Which side of the point's window the cut bounds.
    pub bound: Bound,
    /// The bound, in ticks after the reference.
    pub value: Ticks,
}

impl Cut {
    /// The cut as the constraint `z -> point` it stands for.
    const fn constraint(&self) -> Constraint {
        let (lo, hi) = match self.bound {
            Bound::Lower => (Some(self.value), None),
            Bound::Upper => (None, Some(self.value)),
        };

        Constraint {
            from: REFERENCE,
            to: self.point,
            lo,
            hi,
        }
    }
}

/// A temporal plan shared by several agents, built on the ground.
///
/// Time point `0` is the reference `z` (for example the mission epoch) that every agent
/// knows; every other point belongs to one agent.
///
/// # Examples
///
/// ```
/// use temporal_control::{AgentId, Budget, MultiAgentStn, Ticks, Verdict};
///
/// // Rover 0 drills, rover 1 images the site 5..20 ticks later.
/// let mut plan = MultiAgentStn::new(2);
/// let z = plan.reference();
/// let drill = plan.add_time_point(AgentId(0)).expect("known agent");
/// let image = plan.add_time_point(AgentId(1)).expect("known agent");
/// plan.add_constraint(z, drill, Some(Ticks(0)), Some(Ticks(30))).expect("exists");
/// plan.add_constraint(z, image, Some(Ticks(0)), Some(Ticks(60))).expect("exists");
/// plan.add_constraint(drill, image, Some(Ticks(5)), Some(Ticks(20))).expect("exists");
///
/// let Verdict::Consistent(agents) = plan.decouple(&Budget::UNLIMITED).expect("fits") else {
///     panic!("consistent");
/// };
/// // Each rover now has a window for its own point, and any choice inside both windows
/// // keeps 5 <= image - drill <= 20.
/// assert_eq!(agents[0].cuts().len(), 2);
/// assert_eq!(agents[1].cuts().len(), 2);
/// ```
#[derive(Debug, Clone)]
pub struct MultiAgentStn {
    /// All points and constraints; point `0` is the reference.
    stn: Stn,
    /// Owner of each point, indexed by point; `None` only for the reference.
    owners: Vec<Option<AgentId>>,
    /// Number of agents.
    agents: usize,
}

impl MultiAgentStn {
    /// Creates a plan for `agents` agents with only the reference point.
    #[must_use]
    pub fn new(agents: usize) -> Self {
        let mut stn = Stn::new();
        // The first point of an empty network cannot overflow the count.
        let reference = stn.add_time_point();
        debug_assert_eq!(reference, Ok(REFERENCE));

        Self {
            stn,
            owners: alloc::vec![None],
            agents,
        }
    }

    /// The reference point every agent shares.
    #[must_use]
    pub const fn reference(&self) -> TimePoint {
        REFERENCE
    }

    /// Number of agents.
    #[must_use]
    pub const fn agent_count(&self) -> usize {
        self.agents
    }

    /// Adds a time point owned by `agent`.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownAgent`] if `agent` is not below [`Self::agent_count`].
    /// - [`Error::Overflow`] if the number of time points would exceed `usize::MAX`.
    pub fn add_time_point(&mut self, agent: AgentId) -> Result<TimePoint, Error> {
        if agent.0 >= self.agents {
            return Err(Error::UnknownAgent);
        }

        let tp = self.stn.add_time_point()?;
        self.owners.push(Some(agent));
        Ok(tp)
    }

    /// Adds the constraint `lo <= t_to - t_from <= hi`, local or shared.
    ///
    /// # Errors
    ///
    /// As for [`Stn::add_constraint`].
    pub fn add_constraint(
        &mut self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
    ) -> Result<ConstraintId, Error> {
        self.stn.add_constraint(from, to, lo, hi)
    }

    /// Owner of `tp`; `None` for the reference and for unknown points.
    #[must_use]
    pub fn owner(&self, tp: TimePoint) -> Option<AgentId> {
        self.owners.get(tp.0).copied().flatten()
    }

    /// All points and constraints as one plain [`Stn`], for central checks and explanations.
    #[must_use]
    pub const fn stn(&self) -> &Stn {
        &self.stn
    }

    /// The constraints between points of different agents, in constraint order.
    pub fn shared_constraints(&self) -> impl Iterator<Item = SharedConstraint> + '_ {
        self.stn
            .constraints()
            .iter()
            .enumerate()
            .filter_map(|(i, &constraint)| {
                let from_agent = self.owner(constraint.from)?;
                let to_agent = self.owner(constraint.to)?;

                (from_agent != to_agent).then_some(SharedConstraint {
                    id: ConstraintId(i),
                    constraint,
                    from_agent,
                    to_agent,
                })
            })
    }

    /// Decouples the plan into one [`Agent`] per agent.
    ///
    /// Every combination of schedules that are consistent with the agents'
    /// [`Agent::local_network`]s is consistent with the whole plan. Runs in `O(n^3 + (m + s)
    /// n^2)` steps for `n` points, `m` constraints and `s` shared constraints, with `n^2`
    /// cells; see the [module docs](self) for the choice of the cuts. An inconsistent plan
    /// yields its [`NegativeCycle`](crate::NegativeCycle) over [`Self::stn`].
    ///
    /// # Errors
    ///
    /// - [`Error::MemoryBudgetExceeded`] if `n^2` exceeds [`Budget::max_cells`].
    /// - [`Error::OutOfMemory`] if the allocator refuses the matrix.
    /// - [`Error::BudgetExhausted`] if the step budget runs out.
    /// - [`Error::Overflow`] if a path weight or a cut overflows `i64`.
    /// - [`Error::Internal`] if a cut makes the network inconsistent (a bug).
    pub fn decouple(&self, budget: &Budget) -> Result<Verdict<Vec<Agent>>, Error> {
        let shared: Vec<SharedConstraint> = self.shared_constraints().collect();
        let cuts = match cuts::choose(&self.stn, &shared, budget)? {
            Verdict::Consistent(cuts) => cuts,
            Verdict::Inconsistent(cycle) => return Ok(Verdict::Inconsistent(cycle)),
        };

        let agents = (0..self.agents)
            .map(|a| self.agent(AgentId(a), &shared, &cuts))
            .collect();
        Ok(Verdict::Consistent(agents))
    }

    /// Builds the view of `id`: its points, local constraints, shared constraints and cuts.
    fn agent(&self, id: AgentId, shared: &[SharedConstraint], cuts: &[Cut]) -> Agent {
        let owns = |tp: TimePoint| self.owner(tp) == Some(id);
        let local = |tp: TimePoint| tp == REFERENCE || owns(tp);

        let points = (1..self.owners.len())
            .map(TimePoint)
            .filter(|&tp| owns(tp))
            .collect();
        let constraints = self
            .stn
            .constraints()
            .iter()
            .filter(|c| local(c.from) && local(c.to))
            .copied()
            .collect();
        let shared = shared
            .iter()
            .filter(|s| s.peer(id).is_some())
            .copied()
            .collect();
        let cuts = cuts.iter().filter(|c| owns(c.point)).copied().collect();

        Agent::new(id, points, constraints, shared, cuts)
    }
}
