//! Summaries exchanged between agents, and the joint network a group builds from them.

use alloc::vec::Vec;

use super::{AgentId, REFERENCE, SharedConstraint};
use crate::stn::{Constraint, ConstraintId, NegativeCycle, Stn, Verdict};
use crate::{Budget, Error, TimePoint};

/// What one agent tells the others of a group: its plan projected onto the points they share.
///
/// Made by [`Agent::summary`](super::Agent::summary). Points and constraints are named by
/// their handles in the [`MultiAgentStn`](super::MultiAgentStn).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Summary {
    /// The agent that made the summary.
    agent: AgentId,
    /// The members of the group it was made for, sorted ascending, `agent` included.
    group: Vec<AgentId>,
    /// The agent's [`Agent::revision`](super::Agent::revision) when it made the summary.
    revision: u64,
    /// The agent's points on shared constraints with members, sorted ascending.
    points: Vec<TimePoint>,
    /// The minimal bounds between every pair of the reference and `points`.
    bounds: Vec<Constraint>,
    /// The shared constraints with members, sorted by id.
    shared: Vec<SharedConstraint>,
}

impl Summary {
    /// Assembles a summary; the caller sorts every list.
    pub(super) const fn new(
        agent: AgentId,
        group: Vec<AgentId>,
        revision: u64,
        points: Vec<TimePoint>,
        bounds: Vec<Constraint>,
        shared: Vec<SharedConstraint>,
    ) -> Self {
        Self {
            agent,
            group,
            revision,
            points,
            bounds,
            shared,
        }
    }

    /// The agent that made the summary.
    #[must_use]
    pub const fn agent(&self) -> AgentId {
        self.agent
    }

    /// The members of the group it was made for, sorted ascending.
    #[must_use]
    pub fn group(&self) -> &[AgentId] {
        &self.group
    }

    /// The agent's revision when it made the summary.
    #[must_use]
    pub const fn revision(&self) -> u64 {
        self.revision
    }

    /// The agent's points on shared constraints with members, sorted ascending.
    #[must_use]
    pub fn points(&self) -> &[TimePoint] {
        &self.points
    }

    /// The minimal bounds between the reference and [`Self::points`], pairwise.
    #[must_use]
    pub fn bounds(&self) -> &[Constraint] {
        &self.bounds
    }

    /// The shared constraints with members, sorted by id.
    #[must_use]
    pub fn shared_constraints(&self) -> &[SharedConstraint] {
        &self.shared
    }
}

/// Outcome of a group operation that answered the question.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GroupVerdict {
    /// The group's combined plan is consistent.
    Consistent,
    /// The group's combined plan is inconsistent.
    Inconsistent(GroupConflict),
}

/// Which shared constraints and which agents' plans conflict in a group.
///
/// Summaries hold derived bounds, not the agents' own constraints, so the conflict names the
/// agents whose plans take part rather than their constraints. The plan's own
/// [`Stn::check_consistency`] on [`MultiAgentStn::stn`](super::MultiAgentStn::stn) names the
/// constraints, where the whole plan is available.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GroupConflict {
    /// The shared constraints on the negative cycle, sorted by id.
    pub shared: Vec<ConstraintId>,
    /// The agents whose summarised plans are on the negative cycle, sorted ascending.
    pub agents: Vec<AgentId>,
}

/// Where a constraint of the joint network comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Origin {
    /// A bound of this agent's summary.
    Summary(AgentId),
    /// This shared constraint.
    Shared(ConstraintId),
}

/// The joint network of a group, built from one summary per member.
///
/// Every member that builds it from the same summaries gets the same network, whatever the
/// order of the summaries. Its points are the reference and the points of the summaries.
///
/// # Examples
///
/// ```
/// use temporal_control::{AgentGroup, AgentId, Budget, GroupVerdict, MultiAgentStn, Ticks, Verdict};
///
/// let mut plan = MultiAgentStn::new(2);
/// let z = plan.reference();
/// let a = plan.add_time_point(AgentId(0)).expect("known agent");
/// let b = plan.add_time_point(AgentId(1)).expect("known agent");
/// plan.add_constraint(z, a, Some(Ticks(0)), Some(Ticks(10))).expect("exists");
/// plan.add_constraint(a, b, Some(Ticks(5)), None).expect("exists");
/// let Verdict::Consistent(agents) = plan.decouple(&Budget::UNLIMITED).expect("fits") else {
///     panic!("consistent");
/// };
///
/// let members = [AgentId(0), AgentId(1)];
/// let summaries: Vec<_> = agents
///     .iter()
///     .map(|agent| match agent.summary(&members, &Budget::UNLIMITED).expect("fits") {
///         Verdict::Consistent(summary) => summary,
///         Verdict::Inconsistent(_) => panic!("no proposals"),
///     })
///     .collect();
/// let group = AgentGroup::new(&summaries).expect("complete");
/// assert_eq!(group.check_consistency(&Budget::UNLIMITED), Ok(GroupVerdict::Consistent));
/// ```
#[derive(Debug, Clone)]
pub struct AgentGroup {
    /// The members and their summary revisions, sorted by agent.
    members: Vec<(AgentId, u64)>,
    /// The plan's points in the joint network, sorted; `points[0]` is the reference.
    points: Vec<TimePoint>,
    /// The joint network: every summary bound, then every shared constraint.
    stn: Stn,
    /// The origin of each constraint of `stn`.
    origins: Vec<Origin>,
    /// The members' shared constraints on the points of `stn`, sorted by id.
    shared: Vec<SharedConstraint>,
}

impl AgentGroup {
    /// Builds the joint network of one summary per member.
    ///
    /// # Errors
    ///
    /// - [`Error::MismatchedSummaries`] if `summaries` is empty, two come from the same agent,
    ///   their groups differ from each other or from the set of their agents, two report the
    ///   same point or the same shared constraint differently, or a shared constraint refers
    ///   to a point its owner did not report.
    /// - [`Error::Overflow`] as for [`Stn::add_constraint`].
    pub fn new(summaries: &[Summary]) -> Result<Self, Error> {
        let mut sorted: Vec<&Summary> = summaries.iter().collect();
        sorted.sort_unstable_by_key(|s| s.agent);
        let members = members(&sorted)?;
        let points = joint_points(&sorted)?;
        let shared = joint_shared(&sorted, &points)?;

        let mut group = Self {
            members,
            points,
            stn: Stn::new(),
            origins: Vec::new(),
            shared: Vec::new(),
        };
        group.build(&sorted, shared)?;
        Ok(group)
    }

    /// The members, sorted ascending.
    pub fn members(&self) -> impl Iterator<Item = AgentId> + '_ {
        self.members.iter().map(|&(agent, _)| agent)
    }

    /// The revision of `agent`'s summary, or `None` if `agent` is not a member.
    #[must_use]
    pub fn revision(&self, agent: AgentId) -> Option<u64> {
        self.members
            .binary_search_by_key(&agent, |&(a, _)| a)
            .ok()
            .map(|i| self.members[i].1)
    }

    /// Checks whether the members' plans, with their cuts towards other agents, are jointly
    /// consistent. With every agent of the plan in the group, this is the consistency of the
    /// whole plan.
    ///
    /// Runs Bellman-Ford on the joint network, `O(p * b)` steps for `p` shared points and `b`
    /// bounds.
    ///
    /// # Errors
    ///
    /// As for [`Stn::check_consistency`].
    pub fn check_consistency(&self, budget: &Budget) -> Result<GroupVerdict, Error> {
        Ok(match self.stn.check_consistency(budget)? {
            Verdict::Consistent(_) => GroupVerdict::Consistent,
            Verdict::Inconsistent(cycle) => GroupVerdict::Inconsistent(self.conflict(&cycle)),
        })
    }

    /// The joint network, on points numbered by their position in the sorted point list.
    pub(super) const fn stn(&self) -> &Stn {
        &self.stn
    }

    /// The members' shared constraints on the points of [`Self::stn`].
    pub(super) fn shared(&self) -> &[SharedConstraint] {
        &self.shared
    }

    /// The plan's point for a point of [`Self::stn`].
    pub(super) fn plan_point(&self, joint: TimePoint) -> Option<TimePoint> {
        self.points.get(joint.0).copied()
    }

    /// Names the shared constraints and agents behind a negative cycle of [`Self::stn`].
    pub(super) fn conflict(&self, cycle: &NegativeCycle) -> GroupConflict {
        let mut shared = Vec::new();
        let mut agents = Vec::new();

        for edge in cycle.edges() {
            match self.origins.get(edge.constraint.0) {
                Some(&Origin::Shared(id)) => shared.push(id),
                Some(&Origin::Summary(agent)) => agents.push(agent),
                None => {}
            }
        }

        shared.sort_unstable();
        shared.dedup();
        agents.sort_unstable();
        agents.dedup();
        GroupConflict { shared, agents }
    }

    /// Fills the joint network: the points, every summary bound, then every shared constraint.
    ///
    /// # Errors
    ///
    /// [`Error::MismatchedSummaries`] if a bound names a point no summary reported;
    /// [`Error::Overflow`] as for [`Stn::add_constraint`].
    fn build(&mut self, sorted: &[&Summary], shared: Vec<SharedConstraint>) -> Result<(), Error> {
        for _ in 0..self.points.len() {
            self.stn.add_time_point()?;
        }

        for summary in sorted {
            for bound in &summary.bounds {
                let joint = self.to_joint(bound)?;
                self.push(joint, Origin::Summary(summary.agent))?;
            }
        }

        for s in &shared {
            self.push(s.constraint, Origin::Shared(s.id))?;
        }

        self.shared = shared;
        Ok(())
    }

    /// Adds one constraint on joint points and records its origin.
    ///
    /// # Errors
    ///
    /// As for [`Stn::add_constraint`].
    fn push(&mut self, c: Constraint, origin: Origin) -> Result<(), Error> {
        self.stn.add_constraint(c.from, c.to, c.lo, c.hi)?;
        self.origins.push(origin);
        Ok(())
    }

    /// Renames the points of a plan constraint to joint points.
    ///
    /// # Errors
    ///
    /// [`Error::MismatchedSummaries`] if a point is not in the joint network.
    fn to_joint(&self, c: &Constraint) -> Result<Constraint, Error> {
        Ok(Constraint {
            from: joint_point(&self.points, c.from)?,
            to: joint_point(&self.points, c.to)?,
            ..*c
        })
    }
}

/// The members and their revisions, checking that the summaries form one group.
///
/// # Errors
///
/// [`Error::MismatchedSummaries`] if there are no summaries, two share an agent, or a
/// summary's group is not exactly the set of agents.
fn members(sorted: &[&Summary]) -> Result<Vec<(AgentId, u64)>, Error> {
    let members: Vec<(AgentId, u64)> = sorted.iter().map(|s| (s.agent, s.revision)).collect();
    let agents: Vec<AgentId> = members.iter().map(|&(a, _)| a).collect();
    let distinct = agents.windows(2).all(|w| w[0] < w[1]);

    if agents.is_empty() || !distinct || sorted.iter().any(|s| s.group != agents) {
        return Err(Error::MismatchedSummaries);
    }

    Ok(members)
}

/// The reference and every reported point, sorted.
///
/// # Errors
///
/// [`Error::MismatchedSummaries`] if two summaries report the same point, or one reports the
/// reference.
fn joint_points(sorted: &[&Summary]) -> Result<Vec<TimePoint>, Error> {
    let mut points: Vec<TimePoint> = core::iter::once(REFERENCE)
        .chain(sorted.iter().flat_map(|s| s.points.iter().copied()))
        .collect();
    let reported = points.len();
    points.sort_unstable();
    points.dedup();

    if points.len() != reported {
        return Err(Error::MismatchedSummaries);
    }

    Ok(points)
}

/// The members' shared constraints, once each, sorted by id and renamed to joint points.
///
/// # Errors
///
/// [`Error::MismatchedSummaries`] if two summaries report one id differently, a constraint's
/// agents are not both members, an endpoint is unreported, or a member summarised a shared
/// constraint its peer left out.
fn joint_shared(sorted: &[&Summary], points: &[TimePoint]) -> Result<Vec<SharedConstraint>, Error> {
    let mut all: Vec<SharedConstraint> = sorted
        .iter()
        .flat_map(|s| s.shared.iter().copied())
        .collect();
    all.sort_unstable_by_key(|s| s.id);

    let mut shared: Vec<SharedConstraint> = Vec::new();

    for (i, s) in all.iter().enumerate() {
        let same_as_previous = i > 0 && all[i - 1].id == s.id;

        if same_as_previous {
            if all[i - 1] != *s {
                return Err(Error::MismatchedSummaries);
            }

            continue;
        }

        let both_report = all.get(i + 1).is_some_and(|next| next.id == s.id);

        if !both_report {
            return Err(Error::MismatchedSummaries);
        }

        let constraint = Constraint {
            from: joint_point(points, s.constraint.from)?,
            to: joint_point(points, s.constraint.to)?,
            ..s.constraint
        };
        shared.push(SharedConstraint { constraint, ..*s });
    }

    Ok(shared)
}

/// The joint point of a plan point.
///
/// # Errors
///
/// [`Error::MismatchedSummaries`] if no summary reported `tp`.
fn joint_point(points: &[TimePoint], tp: TimePoint) -> Result<TimePoint, Error> {
    points
        .binary_search(&tp)
        .map(TimePoint)
        .map_err(|_| Error::MismatchedSummaries)
}
