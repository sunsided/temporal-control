//! `Ticks` arithmetic against an `i128` oracle: exact when the result fits, `None` otherwise.

use proptest::prelude::*;
use temporal_control::Ticks;

/// The oracle: `exact` as [`Ticks`] if it fits `i64`, `None` otherwise.
fn fitting(exact: i128) -> Option<Ticks> {
    i64::try_from(exact).ok().map(Ticks)
}

/// Any `i64`, with the overflow boundaries drawn often.
fn any_tick() -> impl Strategy<Value = i64> {
    prop_oneof![
        Just(i64::MIN),
        Just(i64::MIN + 1),
        Just(-1i64),
        Just(0i64),
        Just(1i64),
        Just(i64::MAX - 1),
        Just(i64::MAX),
        any::<i64>(),
    ]
}

proptest! {
    /// `checked_add` is exact or `None` on overflow.
    #[test]
    fn checked_add_matches_oracle(a in any_tick(), b in any_tick()) {
        let exact = i128::from(a) + i128::from(b);
        prop_assert_eq!(Ticks(a).checked_add(Ticks(b)), fitting(exact));
    }

    /// `checked_sub` is exact or `None` on overflow.
    #[test]
    fn checked_sub_matches_oracle(a in any_tick(), b in any_tick()) {
        let exact = i128::from(a) - i128::from(b);
        prop_assert_eq!(Ticks(a).checked_sub(Ticks(b)), fitting(exact));
    }

    /// `checked_neg` is exact, and `None` only for `i64::MIN`.
    #[test]
    fn checked_neg_matches_oracle(a in any_tick()) {
        prop_assert_eq!(Ticks(a).checked_neg(), fitting(-i128::from(a)));
    }
}
