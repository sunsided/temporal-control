//! Ground-side container for a dispatchable network.

use alloc::vec;
use alloc::vec::Vec;

use super::encoding::{self, DecodeError, EncodeError, Encoded};
use super::source::{self, Source};
use super::{DispatchEdge, DispatchError, Dispatcher, Index, PointKind, Wait};

/// A dispatchable network produced on the ground, ready to load into a [`Dispatcher`].
///
/// Produced by [`Stn::dispatchable`](crate::Stn::dispatchable) and
/// [`Stnu::dispatchable`](crate::Stnu::dispatchable). Point indices match the source network.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DispatchableNetwork {
    /// Kind of every time point, indexed by [`TimePoint`](crate::TimePoint).
    points: Vec<PointKind>,
    /// Edges the dispatcher propagates along.
    edges: Vec<DispatchEdge>,
    /// Waits of executable points on contingent points. Empty for STNs.
    waits: Vec<Wait>,
}

impl DispatchableNetwork {
    /// Bundles the parts of a dispatchable network.
    pub(crate) fn new(points: Vec<PointKind>, edges: Vec<DispatchEdge>, waits: Vec<Wait>) -> Self {
        Self {
            points,
            edges,
            waits,
        }
    }

    /// Kind of every time point.
    #[must_use]
    pub fn points(&self) -> &[PointKind] {
        &self.points
    }

    /// Dispatchable edges.
    #[must_use]
    pub fn edges(&self) -> &[DispatchEdge] {
        &self.edges
    }

    /// Waits on contingent points.
    #[must_use]
    pub fn waits(&self) -> &[Wait] {
        &self.waits
    }

    /// Adjacency entries a [`Dispatcher`] needs: two per edge plus one per wait.
    #[must_use]
    pub fn entry_count(&self) -> usize {
        2 * self.edges.len() + self.waits.len()
    }

    /// Loads this network into a new fixed-capacity dispatcher.
    ///
    /// The index type `I` is usually inferred from the annotated result type, e.g.
    /// `let d: Dispatcher<8, 64> = net.dispatcher()?` (with the default `usize`) or
    /// `Dispatcher<8, 64, u16>`. To load into an existing dispatcher in place, use
    /// [`Dispatcher::load`] with [`Self::points`], [`Self::edges`] and [`Self::waits`].
    ///
    /// # Errors
    ///
    /// As for [`Dispatcher::load`].
    pub fn dispatcher<const N: usize, const E: usize, I: Index>(
        &self,
    ) -> Result<Dispatcher<N, E, I>, DispatchError> {
        Dispatcher::new(&self.points, &self.edges, &self.waits)
    }

    /// Encodes this network in the compact checksummed format of [`encoding`].
    ///
    /// # Examples
    ///
    /// ```
    /// use temporal_control::{Budget, Dispatcher, DispatchableNetwork, Stn, Ticks, Verdict};
    ///
    /// let mut stn = Stn::new();
    /// let a = stn.add_time_point().expect("fits");
    /// let b = stn.add_time_point().expect("fits");
    /// stn.add_constraint(a, b, Some(Ticks(5)), Some(Ticks(15))).expect("exists");
    /// let Verdict::Consistent(net) = stn.dispatchable(&Budget::UNLIMITED).expect("fits") else {
    ///     panic!("consistent");
    /// };
    ///
    /// let bytes = net.encode().expect("small");
    /// assert_eq!(DispatchableNetwork::decode(&bytes), Ok(net));
    ///
    /// let mut d = Dispatcher::<2, 8>::EMPTY;
    /// d.load_encoded(&bytes).expect("valid");
    /// ```
    ///
    /// # Errors
    ///
    /// [`EncodeError::TooLarge`] if a count or index exceeds `u32::MAX`.
    pub fn encode(&self) -> Result<Vec<u8>, EncodeError> {
        let len = encoding::encoded_len(self.points.len(), self.edges.len(), self.waits.len())
            .ok_or(EncodeError::TooLarge)?;
        let mut out = vec![0; len];
        encoding::encode_into(&self.points, &self.edges, &self.waits, &mut out)?;
        Ok(out)
    }

    /// Decodes and validates an encoding, for ground-side checks of what will be uplinked.
    ///
    /// Accepts exactly the encodings that a dispatcher of unlimited capacity with `usize`
    /// indices accepts.
    ///
    /// # Errors
    ///
    /// As for [`Dispatcher::load_encoded`], except that capacities never apply.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let encoded = Encoded::parse(bytes)?;
        source::validate::<usize>(&encoded, usize::MAX, usize::MAX)
            .map_err(DecodeError::Network)?;

        let invalid = DecodeError::Network(DispatchError::InvalidNetwork);
        let points = (0..encoded.point_count()).map(|i| encoded.point(i));
        let edges = (0..encoded.edge_count()).map(|i| encoded.edge(i));
        let waits = (0..encoded.wait_count()).map(|i| encoded.wait(i));

        Ok(Self::new(
            points.collect::<Option<_>>().ok_or(invalid)?,
            edges.collect::<Option<_>>().ok_or(invalid)?,
            waits.collect::<Option<_>>().ok_or(invalid)?,
        ))
    }
}
