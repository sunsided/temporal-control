//! Reaction-time variants: same-tick and next-tick reaction to observations.

use super::Stnu;
use crate::{Error, Ticks, TimePoint};

/// When the agent may react to an observation of a contingent point.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum Reaction {
    /// An observation at tick `t` may influence executions at `t` (the crate's semantics).
    #[default]
    SameTick,
    /// An observation at tick `t` may only influence executions from `t + 1` on: decisions
    /// at `t` see contingent events strictly before `t`.
    NextTick,
}

impl Stnu {
    /// The network whose same-tick dynamic controllability equals this network's under
    /// `reaction`.
    ///
    /// For [`Reaction::SameTick`] this is a copy. For [`Reaction::NextTick`] every contingent
    /// point `C` is replaced by its *observation* `C' = C + 1`, which the agent sees at tick
    /// `C'`, i.e. one tick after `C` happened: a link from an executable point becomes
    /// `[lo + 1, hi + 1]`, a link from a contingent point keeps its bounds (both ends shift),
    /// and every requirement is rewritten with `C = C' - 1`. Under same-tick reaction an agent
    /// of the new network sees exactly what a next-tick agent of the old one sees, so the
    /// transformation is exact; the tests check it against the brute-force game with the
    /// next-tick move order. Point and constraint indices are unchanged, so a
    /// [`SemiReducibleCycle`](super::SemiReducibleCycle) of the result names the same
    /// constraints and links.
    ///
    /// Dispatching the result needs an executive that reports each contingent event one tick
    /// late (`C` observed at `t` is handed to the dispatcher at `t + 1`); the dispatcher itself
    /// is unchanged.
    ///
    /// # Examples
    ///
    /// ```
    /// use temporal_control::{Budget, Controllability, Reaction, Stnu, Ticks};
    ///
    /// // X exactly at the end of a [1, 10] activity: needs same-tick reaction.
    /// let mut stnu = Stnu::new();
    /// let a = stnu.add_time_point().expect("fits");
    /// let c = stnu.add_contingent_link(a, Ticks(1), Ticks(10)).expect("valid");
    /// let x = stnu.add_time_point().expect("fits");
    /// stnu.add_constraint(c, x, Some(Ticks(0)), Some(Ticks(0))).expect("exists");
    ///
    /// let dc = |s: &Stnu| {
    ///     let verdict = s.check_controllability(&Budget::UNLIMITED).expect("fits");
    ///     matches!(verdict, Controllability::Controllable(()))
    /// };
    /// assert!(dc(&stnu));
    /// assert!(!dc(&stnu.with_reaction(Reaction::NextTick).expect("no overflow")));
    /// ```
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] if a shifted bound overflows `i64`.
    pub fn with_reaction(&self, reaction: Reaction) -> Result<Self, Error> {
        if reaction == Reaction::SameTick {
            return Ok(self.clone());
        }

        let n = self.time_point_count();
        let mut link_of = alloc::vec![None; n];

        for (i, link) in self.links.iter().enumerate() {
            link_of[link.contingent.0] = Some(i);
        }

        let shift = |p: TimePoint| i64::from(link_of[p.0].is_some());
        let mut out = Self::new();

        for link in &link_of {
            let Some(i) = *link else {
                out.add_time_point()?;
                continue;
            };

            let link = self.links[i];
            let d = Ticks(1 - shift(link.activation));
            let lo = link.lo.checked_add(d).ok_or(Error::Overflow)?;
            let hi = link.hi.checked_add(d).ok_or(Error::Overflow)?;
            out.add_contingent_link(link.activation, lo, hi)?;
        }

        for c in self.requirements.constraints() {
            let d = Ticks(shift(c.to) - shift(c.from));
            let moved = |b: Option<Ticks>| {
                b.map(|b| b.checked_add(d).ok_or(Error::Overflow))
                    .transpose()
            };
            out.add_constraint(c.from, c.to, moved(c.lo)?, moved(c.hi)?)?;
        }

        Ok(out)
    }
}
