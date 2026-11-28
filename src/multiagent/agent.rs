//! One agent's view of a decoupled plan.

use alloc::vec::Vec;

use super::group::{AgentGroup, GroupVerdict, Summary};
use super::{AgentId, Cut, REFERENCE, SharedConstraint, cuts};
use crate::stn::{Constraint, Stn, Verdict};
use crate::{Budget, Error, Ticks, TimePoint};

/// The part of a multi-agent plan one agent holds: its points, local constraints, shared
/// constraints and the cuts that decouple them.
///
/// Every point is named by its handle in the [`MultiAgentStn`](super::MultiAgentStn). Its
/// [`Self::local_network`] is always consistent, and every schedule of it combines with any
/// schedules of the other agents' local networks into a schedule of the whole plan, as long
/// as the agents adopt re-decouplings together (see the [module docs](super)).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Agent {
    /// This agent's handle.
    id: AgentId,
    /// The points this agent owns, sorted ascending; the reference is not among them.
    points: Vec<TimePoint>,
    /// Constraints among the reference and [`Self::points`], in the order they were added.
    constraints: Vec<Constraint>,
    /// Proposed constraints, among the same points, that the next re-decoupling adopts or
    /// drops. Not part of the local network.
    pending: Vec<Constraint>,
    /// The shared constraints with a point of this agent, sorted by id.
    shared: Vec<SharedConstraint>,
    /// The cuts on this agent's points.
    cuts: Vec<Cut>,
    /// Counts changes to `constraints`, `pending` and `cuts`, starting at zero.
    revision: u64,
}

/// The stored state of an [`Agent`], as plain data, for moving an agent between processes.
///
/// The ground decouples a plan and sends each agent its part; the agent's own computer
/// rebuilds it with [`Agent::from_parts`]. Every field mirrors the accessor of the same name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AgentParts {
    /// The agent's handle.
    pub id: AgentId,
    /// The points it owns, sorted ascending, without the reference.
    pub points: Vec<TimePoint>,
    /// Its local constraints, among the reference and `points`.
    pub constraints: Vec<Constraint>,
    /// Its proposed constraints, among the same points.
    pub pending: Vec<Constraint>,
    /// The shared constraints with one of its points, sorted by id.
    pub shared: Vec<SharedConstraint>,
    /// The cuts on its points, each of a constraint in `shared`.
    pub cuts: Vec<Cut>,
    /// Its revision.
    pub revision: u64,
}

impl Agent {
    /// Rebuilds an agent from its parts, as received from the ground or another process.
    ///
    /// Checks the structure, then that the local network is consistent; a negative verdict
    /// explains the inconsistency over the constraints of [`Self::local_network`].
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if `points` is not sorted without duplicates or contains
    ///   the reference, or a constraint, proposal, shared constraint or cut names a point the
    ///   agent does not own.
    /// - [`Error::UnknownAgent`] if a shared constraint does not involve the agent, `shared` is
    ///   not sorted by id without duplicates, or a cut refers to a constraint not in `shared`.
    /// - [`Error::Overflow`] if a lower bound is `i64::MIN`; otherwise any error of
    ///   [`Stn::check_consistency`].
    pub fn from_parts(parts: AgentParts, budget: &Budget) -> Result<Verdict<Self>, Error> {
        let agent = Self {
            id: parts.id,
            points: parts.points,
            constraints: parts.constraints,
            pending: parts.pending,
            shared: parts.shared,
            cuts: parts.cuts,
            revision: parts.revision,
        };
        agent.validate()?;

        Ok(match agent.local_network()?.check_consistency(budget)? {
            Verdict::Consistent(_) => Verdict::Consistent(agent),
            Verdict::Inconsistent(cycle) => Verdict::Inconsistent(cycle),
        })
    }

    /// The stored state as plain data; the inverse of [`Self::from_parts`].
    #[must_use]
    pub fn into_parts(self) -> AgentParts {
        AgentParts {
            id: self.id,
            points: self.points,
            constraints: self.constraints,
            pending: self.pending,
            shared: self.shared,
            cuts: self.cuts,
            revision: self.revision,
        }
    }

    /// Checks the structural invariants [`Self::from_parts`] documents.
    ///
    /// # Errors
    ///
    /// As for [`Self::from_parts`], without the consistency check.
    fn validate(&self) -> Result<(), Error> {
        let sorted_points = self.points.windows(2).all(|w| w[0] < w[1]);

        if !sorted_points || self.points.first() == Some(&REFERENCE) {
            return Err(Error::UnknownTimePoint);
        }

        for c in self.constraints.iter().chain(&self.pending) {
            self.local_pair(c.from, c.to)?;

            if c.lo.is_some_and(|l| l.checked_neg().is_none()) {
                return Err(Error::Overflow);
            }
        }

        self.validate_shared()?;

        for cut in &self.cuts {
            if self.local_point(cut.point).is_none_or(|tp| tp == REFERENCE) {
                return Err(Error::UnknownTimePoint);
            }

            if self
                .shared
                .binary_search_by_key(&cut.shared, |s| s.id)
                .is_err()
            {
                return Err(Error::UnknownAgent);
            }
        }

        Ok(())
    }

    /// Checks that every shared constraint involves this agent at one of its own points.
    ///
    /// # Errors
    ///
    /// As for [`Self::from_parts`].
    fn validate_shared(&self) -> Result<(), Error> {
        if !self.shared.windows(2).all(|w| w[0].id < w[1].id) {
            return Err(Error::UnknownAgent);
        }

        for s in &self.shared {
            let own = if s.from_agent == self.id {
                s.constraint.from
            } else if s.to_agent == self.id {
                s.constraint.to
            } else {
                return Err(Error::UnknownAgent);
            };

            let foreign = self.local_point(own).is_none_or(|tp| tp == REFERENCE);

            if s.from_agent == s.to_agent || foreign {
                return Err(Error::UnknownTimePoint);
            }
        }

        Ok(())
    }

    /// Creates the view; `points` and `shared` come sorted from the plan.
    pub(super) const fn new(
        id: AgentId,
        points: Vec<TimePoint>,
        constraints: Vec<Constraint>,
        shared: Vec<SharedConstraint>,
        cuts: Vec<Cut>,
    ) -> Self {
        Self {
            id,
            points,
            constraints,
            pending: Vec::new(),
            shared,
            cuts,
            revision: 0,
        }
    }

    /// This agent's handle.
    #[must_use]
    pub const fn id(&self) -> AgentId {
        self.id
    }

    /// The points this agent owns, sorted ascending, without the reference.
    #[must_use]
    pub fn points(&self) -> &[TimePoint] {
        &self.points
    }

    /// The local constraints, among the reference and [`Self::points`].
    #[must_use]
    pub fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    /// The proposed constraints the next re-decoupling adopts or drops.
    #[must_use]
    pub fn pending(&self) -> &[Constraint] {
        &self.pending
    }

    /// The shared constraints with a point of this agent, sorted by id.
    #[must_use]
    pub fn shared_constraints(&self) -> &[SharedConstraint] {
        &self.shared
    }

    /// The cuts on this agent's points.
    #[must_use]
    pub fn cuts(&self) -> &[Cut] {
        &self.cuts
    }

    /// Counts the changes to the local and proposed constraints and the cuts. A [`Summary`]
    /// carries the revision it was made from.
    #[must_use]
    pub const fn revision(&self) -> u64 {
        self.revision
    }

    /// The point of [`Self::local_network`] that stands for `tp`: the reference is `0`, and
    /// `points()[i]` is `i + 1`. `None` if this agent does not own `tp`.
    #[must_use]
    pub fn local_point(&self, tp: TimePoint) -> Option<TimePoint> {
        if tp == REFERENCE {
            return Some(REFERENCE);
        }

        self.points
            .binary_search(&tp)
            .ok()
            .map(|i| TimePoint(i + 1))
    }

    /// The plan's point for a point of [`Self::local_network`]; the inverse of
    /// [`Self::local_point`].
    #[must_use]
    pub fn plan_point(&self, local: TimePoint) -> Option<TimePoint> {
        match local.0 {
            0 => Some(REFERENCE),
            i => self.points.get(i - 1).copied(),
        }
    }

    /// The network this agent dispatches alone: its local constraints, then its cuts.
    /// Proposed constraints are not part of it.
    ///
    /// Points are numbered as in [`Self::local_point`]. Dispatch it with
    /// [`Stn::dispatchable`], with the reference executed at the plan's epoch.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if the point count overflows (not reachable from a plan).
    pub fn local_network(&self) -> Result<Stn, Error> {
        self.network(false, |_| true)
    }

    /// Adds the local constraint `lo <= t_to - t_from <= hi` if the local network stays
    /// consistent, without talking to any other agent.
    ///
    /// The cuts guarantee that a locally consistent change cannot break the plan. A rejected
    /// constraint leaves the agent unchanged; its explanation refers to the constraints of
    /// [`Self::local_network`] with the rejected one appended last. A rejected constraint may
    /// still fit the plan with other cuts: [`Self::propose`] it for the next re-decoupling.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if `from` or `to` is neither the reference nor a point of
    ///   this agent.
    /// - [`Error::Overflow`] if `lo` is `i64::MIN` or a path weight overflows.
    /// - [`Error::BudgetExhausted`] if the consistency check runs out of steps.
    pub fn add_constraint(
        &mut self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
        budget: &Budget,
    ) -> Result<Verdict<()>, Error> {
        let (local_from, local_to) = self.local_pair(from, to)?;
        let mut candidate = self.local_network()?;
        candidate.add_constraint(local_from, local_to, lo, hi)?;

        if let Verdict::Inconsistent(cycle) = candidate.check_consistency(budget)? {
            return Ok(Verdict::Inconsistent(cycle));
        }

        self.constraints.push(Constraint { from, to, lo, hi });
        self.revision = self.revision.saturating_add(1);
        Ok(Verdict::Consistent(()))
    }

    /// Proposes the local constraint `lo <= t_to - t_from <= hi` for the next re-decoupling.
    ///
    /// Use this for a change the cuts do not admit ([`Self::add_constraint`] rejected it).
    /// The proposal does not constrain [`Self::local_network`]. It enters this agent's next
    /// [`Self::summary`], so the group can check whether it fits the plan
    /// ([`AgentGroup::check_consistency`]); [`Self::redecouple`] then adopts it, or drops it if
    /// the group's plan is inconsistent.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownTimePoint`] if `from` or `to` is neither the reference nor a point of
    ///   this agent.
    /// - [`Error::Overflow`] if `lo` is `i64::MIN`, whose negation is not representable.
    pub fn propose(
        &mut self,
        from: TimePoint,
        to: TimePoint,
        lo: Option<Ticks>,
        hi: Option<Ticks>,
    ) -> Result<(), Error> {
        self.local_pair(from, to)?;

        if lo.is_some_and(|l| l.checked_neg().is_none()) {
            return Err(Error::Overflow);
        }

        self.pending.push(Constraint { from, to, lo, hi });
        self.revision = self.revision.saturating_add(1);
        Ok(())
    }

    /// Summarises this agent for a re-decoupling or a joint check with the agents of `group`.
    ///
    /// The summary covers the local and proposed constraints. It keeps the cuts of shared
    /// constraints with agents outside `group` and drops those with members, whose slack the
    /// group may redistribute. It holds the minimal bounds between the reference and every
    /// point of a shared constraint with a member, plus those shared constraints. Costs
    /// `O(n^3)` steps and `n^2` cells for the `n` local points.
    ///
    /// A negative verdict means the proposals conflict with the local constraints and the
    /// kept cuts; its explanation refers to the constraints, then the proposals, then the kept
    /// cuts, on the points of [`Self::local_point`]. Without proposals it is always positive.
    ///
    /// Pin every point that has already happened with a local constraint `z -> tp` in
    /// `[t, t]` ([`Self::add_constraint`]) before summarising, so that no new cut contradicts
    /// the past. Do not change the agent between sending the summary and
    /// [`Self::redecouple`]: a change bumps [`Self::revision`], and the agent then refuses the
    /// group.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownAgent`] if `group` does not contain this agent.
    /// - [`Error::MemoryBudgetExceeded`], [`Error::OutOfMemory`], [`Error::BudgetExhausted`],
    ///   [`Error::Overflow`] or [`Error::Internal`] as for [`Stn::minimal_network`].
    pub fn summary(&self, group: &[AgentId], budget: &Budget) -> Result<Verdict<Summary>, Error> {
        let mut group = group.to_vec();
        group.sort_unstable();
        group.dedup();

        if group.binary_search(&self.id).is_err() {
            return Err(Error::UnknownAgent);
        }

        let member = |s: &SharedConstraint| {
            s.peer(self.id)
                .is_some_and(|p| group.binary_search(&p).is_ok())
        };
        let shared: Vec<SharedConstraint> =
            self.shared.iter().filter(|s| member(s)).copied().collect();
        let network = self.network(true, |cut| !self.shared_with(cut, member))?;

        let minimal = match network.minimal_network(budget)? {
            Verdict::Consistent(minimal) => minimal,
            Verdict::Inconsistent(cycle) => return Ok(Verdict::Inconsistent(cycle)),
        };

        let interface = self.interface(&shared);
        let bounds = interface_bounds(
            &interface,
            |tp| self.local_point(tp),
            |a, b| (minimal.lower(a, b), minimal.upper(a, b)),
        );

        Ok(Verdict::Consistent(Summary::new(
            self.id,
            group,
            self.revision,
            interface,
            bounds,
            shared,
        )))
    }

    /// Replaces the cuts of the group's shared constraints with the group's new decoupling.
    ///
    /// Every member that calls this with the same `group` gets its side of the same cuts, and
    /// adopts its proposals into its local constraints. A negative verdict means the group's
    /// plan, proposals included and with the cuts towards other agents fixed, is inconsistent;
    /// the agent then drops its proposals and keeps its cuts. Read the protocol requirement in
    /// the [module docs](super) before using this: if any member refuses the group, every
    /// member must stay on (or return to) its old cuts, for example from a clone taken
    /// before the call.
    ///
    /// # Errors
    ///
    /// - [`Error::MismatchedSummaries`] if this agent is not a member of `group`, or its
    ///   summary in `group` is older than [`Self::revision`].
    /// - Any error of [`MultiAgentStn::decouple`](super::MultiAgentStn::decouple) on the
    ///   group's network.
    pub fn redecouple(
        &mut self,
        group: &AgentGroup,
        budget: &Budget,
    ) -> Result<GroupVerdict, Error> {
        if group.revision(self.id) != Some(self.revision) {
            return Err(Error::MismatchedSummaries);
        }

        let new_cuts = match cuts::choose(group.stn(), group.shared(), budget)? {
            Verdict::Consistent(cuts) => cuts,
            Verdict::Inconsistent(cycle) => {
                let conflict = group.conflict(&cycle)?;
                self.pending.clear();
                self.revision = self.revision.saturating_add(1);
                return Ok(GroupVerdict::Inconsistent(conflict));
            }
        };

        self.constraints.append(&mut self.pending);

        let member =
            |s: &SharedConstraint| s.peer(self.id).is_some_and(|p| group.revision(p).is_some());
        let mut cuts = core::mem::take(&mut self.cuts);
        cuts.retain(|cut| !self.shared_with(cut, member));
        self.cuts = cuts;

        for cut in new_cuts {
            let point = group.plan_point(cut.point).ok_or(Error::Internal)?;

            if self.local_point(point).is_some_and(|tp| tp != REFERENCE) {
                self.cuts.push(Cut { point, ..cut });
            }
        }

        self.revision = self.revision.saturating_add(1);
        Ok(GroupVerdict::Consistent)
    }

    /// The local points of `from` and `to`.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownTimePoint`] if either is neither the reference nor a point of this agent.
    fn local_pair(&self, from: TimePoint, to: TimePoint) -> Result<(TimePoint, TimePoint), Error> {
        let local = |tp| self.local_point(tp).ok_or(Error::UnknownTimePoint);
        Ok((local(from)?, local(to)?))
    }

    /// Whether `cut` belongs to a shared constraint for which `pick` holds.
    fn shared_with(&self, cut: &Cut, pick: impl Fn(&SharedConstraint) -> bool) -> bool {
        self.shared
            .binary_search_by_key(&cut.shared, |s| s.id)
            .is_ok_and(|i| pick(&self.shared[i]))
    }

    /// The own points of `shared`, sorted ascending and without duplicates.
    fn interface(&self, shared: &[SharedConstraint]) -> Vec<TimePoint> {
        let mut points: Vec<TimePoint> = shared
            .iter()
            .flat_map(|s| [s.constraint.from, s.constraint.to])
            .filter(|&tp| tp != REFERENCE && self.points.binary_search(&tp).is_ok())
            .collect();
        points.sort_unstable();
        points.dedup();
        points
    }

    /// The local network with the local constraints, the proposals if `pending`, and the cuts
    /// for which `keep` holds, in that order.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if the point count overflows; [`Error::Internal`] if a stored
    /// constraint names a point this agent does not own.
    fn network(&self, pending: bool, keep: impl Fn(&Cut) -> bool) -> Result<Stn, Error> {
        let mut stn = Stn::new();

        for _ in 0..=self.points.len() {
            stn.add_time_point()?;
        }

        let cuts = self
            .cuts
            .iter()
            .filter(|cut| keep(cut))
            .map(Cut::constraint);

        let proposals = self.pending.iter().filter(|_| pending);

        for c in self
            .constraints
            .iter()
            .chain(proposals)
            .copied()
            .chain(cuts)
        {
            let from = self.local_point(c.from).ok_or(Error::Internal)?;
            let to = self.local_point(c.to).ok_or(Error::Internal)?;
            stn.add_constraint(from, to, c.lo, c.hi)?;
        }

        Ok(stn)
    }
}

/// The bounds between every pair of the reference and `interface`, as plan constraints.
///
/// `local` maps a plan point to the network `bounds` reads; pairs without any finite bound
/// are left out.
fn interface_bounds(
    interface: &[TimePoint],
    local: impl Fn(TimePoint) -> Option<TimePoint>,
    bounds: impl Fn(TimePoint, TimePoint) -> (Option<Ticks>, Option<Ticks>),
) -> Vec<Constraint> {
    let points: Vec<TimePoint> = core::iter::once(REFERENCE)
        .chain(interface.iter().copied())
        .collect();
    let mut out = Vec::new();

    for (i, &from) in points.iter().enumerate() {
        for &to in &points[i + 1..] {
            let (Some(a), Some(b)) = (local(from), local(to)) else {
                continue;
            };

            let (lo, hi) = bounds(a, b);

            if lo.is_some() || hi.is_some() {
                out.push(Constraint { from, to, lo, hi });
            }
        }
    }

    out
}
