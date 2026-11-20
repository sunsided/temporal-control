//! Examples taken from the papers the algorithms implement, with the answers the papers state.
//!
//! Each test names its source. Where a paper reasons over real-valued time, the example only
//! uses integer bounds and its stated answer holds for integer schedules too.

use temporal_control::{
    Budget, ContingentId, Controllability, DcAlgorithm, DispatchAlgorithm, LabelledEdge, Reaction,
    Stn, Stnu, Ticks, TimePoint, Verdict,
};

use crate::stnu::assert_cycle_closes;

/// Every checker behind [`Stnu::check_controllability_with`].
const CHECKERS: [DcAlgorithm; 4] = [
    DcAlgorithm::Morris2014,
    DcAlgorithm::RulMinus,
    DcAlgorithm::Rul2021,
    DcAlgorithm::PerLabelBackprop,
];

/// Every algorithm behind [`Stnu::dispatchable_with`].
const DISPATCHERS: [DispatchAlgorithm; 3] = [
    DispatchAlgorithm::Minimal,
    DispatchAlgorithm::Morris2014,
    DispatchAlgorithm::Closure,
];

/// Whether every checker finds `stnu` DC; panics if they disagree.
fn all_checkers_dc(stnu: &Stnu) -> bool {
    let verdicts: Vec<bool> = CHECKERS
        .iter()
        .map(|&a| {
            let v = stnu.check_controllability_with(a, &Budget::UNLIMITED);
            matches!(v.expect("fits"), Controllability::Controllable(()))
        })
        .collect();
    assert!(verdicts.iter().all(|&v| v == verdicts[0]), "{verdicts:?}");
    verdicts[0]
}

/// Adds `n` executable time points to `stnu`.
fn points<const N: usize>(stnu: &mut Stnu) -> [TimePoint; N] {
    [(); N].map(|()| stnu.add_time_point().expect("fits"))
}

/// The STP of Dechter, Meiri and Pearl (1991, Example 1.1 and Fig. 3): `x0` is 7:00, John
/// leaves home at `x1` and arrives at `x2`, Fred leaves at `x3` and arrives at `x4`.
fn dmp_example() -> Stn {
    let mut stn = Stn::new();
    let x = [(); 5].map(|()| stn.add_time_point().expect("fits"));
    let c = |stn: &mut Stn, i: usize, j: usize, lo: i64, hi: i64| {
        stn.add_constraint(x[i], x[j], Some(Ticks(lo)), Some(Ticks(hi)))
            .expect("exists");
    };
    c(&mut stn, 0, 1, 10, 20);
    c(&mut stn, 1, 2, 30, 40);
    c(&mut stn, 3, 2, 10, 20);
    c(&mut stn, 3, 4, 40, 50);
    c(&mut stn, 0, 4, 60, 70);
    stn
}

/// Dechter, Meiri and Pearl (1991), Table 1: shortest-path lengths `d[i][j]` of Fig. 3.
const DMP_TABLE_1: [[i64; 5]; 5] = [
    [0, 20, 50, 30, 70],
    [-10, 0, 40, 20, 60],
    [-40, -30, 0, -10, 30],
    [-20, -10, 20, 0, 50],
    [-60, -50, -20, -40, 0],
];

/// Floyd-Warshall reproduces Table 1 of Dechter, Meiri and Pearl (1991).
#[test]
fn dmp_table_1_minimal_network() {
    let Verdict::Consistent(min) = dmp_example().minimal_network(&Budget::UNLIMITED).unwrap()
    else {
        panic!("the example is consistent");
    };

    for (i, row) in DMP_TABLE_1.iter().enumerate() {
        for (j, &d) in row.iter().enumerate() {
            assert_eq!(
                min.upper(TimePoint(i), TimePoint(j)),
                Some(Ticks(d)),
                "d[{i}][{j}]"
            );
        }
    }
}

/// Dechter, Meiri and Pearl (1991, after Table 2): John by bus (`x2 - x1 >= 60`) and Fred in
/// the carpool (`x4 - x3` in `[40, 50]`) is inconsistent.
#[test]
fn dmp_bus_and_carpool_is_inconsistent() {
    let mut stn = Stn::new();
    let x = [(); 5].map(|()| stn.add_time_point().expect("fits"));
    let mut c = |i: usize, j: usize, lo: i64, hi: Option<i64>| {
        stn.add_constraint(x[i], x[j], Some(Ticks(lo)), hi.map(Ticks))
            .expect("exists");
    };
    c(0, 1, 10, Some(20));
    c(1, 2, 60, None);
    c(3, 2, 10, Some(20));
    c(3, 4, 40, Some(50));
    c(0, 4, 60, Some(70));

    let verdict = stn.check_consistency(&Budget::UNLIMITED).unwrap();
    assert!(matches!(verdict, Verdict::Inconsistent(_)));
}

/// The counter-example `N!` of Hunsberger (TIME 2009, Fig. 2): `Z => C0` in `[2, 3]`,
/// `Z => C` in `[10, 20]`, `C - B` in `[10, 15]`, everything after `Z`. There is no safe time
/// for `B`; only a strategy with advance knowledge of `C` (which the original MMV definition
/// failed to exclude) could satisfy it. Not DC under either reaction semantics.
#[test]
fn hunsberger_2009_counter_example_is_not_dc() {
    let mut stnu = Stnu::new();
    let [z] = points(&mut stnu);
    stnu.add_contingent_link(z, Ticks(2), Ticks(3)).unwrap();
    let c = stnu.add_contingent_link(z, Ticks(10), Ticks(20)).unwrap();
    let [b] = points(&mut stnu);
    stnu.add_constraint(b, c, Some(Ticks(10)), Some(Ticks(15)))
        .unwrap();
    stnu.add_constraint(z, b, Some(Ticks(0)), None).unwrap();

    for reaction in [Reaction::SameTick, Reaction::NextTick] {
        let net = stnu.with_reaction(reaction).unwrap();
        assert!(!all_checkers_dc(&net), "{reaction:?}");
    }
}

/// The triangle of Hunsberger (TIME 2009, Fig. 1): `A => C` in `[10, 20]` and a requirement
/// `C - B` in `[lo, hi]`, with `B` after `A`.
fn hunsberger_2009_triangle(lo: i64, hi: i64) -> (Stnu, [TimePoint; 3]) {
    let mut stnu = Stnu::new();
    let [a] = points(&mut stnu);
    let c = stnu.add_contingent_link(a, Ticks(10), Ticks(20)).unwrap();
    let [b] = points(&mut stnu);
    stnu.add_constraint(b, c, Some(Ticks(lo)), Some(Ticks(hi)))
        .unwrap();
    stnu.add_constraint(a, b, Some(Ticks(0)), None).unwrap();
    (stnu, [a, c, b])
}

/// Hunsberger (TIME 2009, Fig. 1a), the precede case: `C - B` in `[2, 15]` forces `B - A` into
/// `[5, 8]`, which the Morris 2014 dispatchable network states as two ordinary edges.
#[test]
fn hunsberger_2009_precede_case_tightens() {
    let (stnu, [a, _, b]) = hunsberger_2009_triangle(2, 15);
    assert!(all_checkers_dc(&stnu));

    let Controllability::Controllable(net) = stnu
        .dispatchable_with(DispatchAlgorithm::Morris2014, &Budget::UNLIMITED)
        .unwrap()
    else {
        panic!("DC");
    };
    let edge = |from, to| {
        net.edges()
            .iter()
            .find(|e| e.from == from && e.to == to)
            .map(|e| e.weight)
    };
    assert_eq!(edge(a, b), Some(Ticks(8)));
    assert_eq!(edge(b, a), Some(Ticks(-5)));
}

/// Hunsberger (TIME 2009, Fig. 1b), the unordered case: `C - B` in `[-4, 7]` makes `B` wait
/// until `A + 13` unless `C` happened first.
#[test]
fn hunsberger_2009_unordered_case_waits() {
    let (stnu, [_, c, b]) = hunsberger_2009_triangle(-4, 7);
    assert!(all_checkers_dc(&stnu));

    for algorithm in DISPATCHERS {
        let Controllability::Controllable(net) = stnu
            .dispatchable_with(algorithm, &Budget::UNLIMITED)
            .unwrap()
        else {
            panic!("DC");
        };
        let wait = net
            .waits()
            .iter()
            .find(|w| w.waiter == b && w.contingent == c);
        assert_eq!(wait.map(|w| w.delay), Some(Ticks(13)), "{algorithm:?}");
    }
}

/// The quiescent non-DC network of Morris, Muscettola and Vidal (IJCAI 2001, Fig. 2), which
/// the local 3DC propagation accepts: `A => C`, `D => B`, `E => F` in `[2, 4]`, `D - A` and
/// `E - D` in `[0, 1]`, `C - B` and `B - F` in `[-3, 1]`, and, if `with_ae`, `E - A` in
/// `[0, 1]`.
#[expect(clippy::many_single_char_names, reason = "the paper's point names")]
fn mmv_2001_fig_2(with_ae: bool) -> Stnu {
    let mut stnu = Stnu::new();
    let [a] = points(&mut stnu);
    let c = stnu.add_contingent_link(a, Ticks(2), Ticks(4)).unwrap();
    let [d] = points(&mut stnu);
    let b = stnu.add_contingent_link(d, Ticks(2), Ticks(4)).unwrap();
    let [e] = points(&mut stnu);
    let f = stnu.add_contingent_link(e, Ticks(2), Ticks(4)).unwrap();
    let unit = (Some(Ticks(0)), Some(Ticks(1)));
    let loose = (Some(Ticks(-3)), Some(Ticks(1)));
    let mut constraints = vec![(a, d, unit), (d, e, unit), (b, c, loose), (f, b, loose)];

    if with_ae {
        constraints.push((a, e, unit));
    }

    for (from, to, (lo, hi)) in constraints {
        stnu.add_constraint(from, to, lo, hi).unwrap();
    }

    stnu
}

/// MMV 2001, Fig. 2: a dynamic strategy needs `D - A = 1` and `E - D = 1`, which breaks
/// `E - A <= 1`; without that constraint the network is DC.
#[test]
fn mmv_2001_quiescent_network_is_not_dc() {
    assert!(!all_checkers_dc(&mmv_2001_fig_2(true)));
    assert!(all_checkers_dc(&mmv_2001_fig_2(false)));
}

/// The cycle Morris (CPAIOR 2014, section 4) uses to illustrate the cubic algorithm, with
/// lower bounds 1 instead of normal form's 0 (each lower-case edge then weighs 1 and each
/// upper-case edge 1 less, so every sub-path keeps its weight):
/// `E -4-> B -B:-3-> A -b:1-> B -1-> D -D:-4-> C -d:1-> D -3-> B -B:-3-> A -b:1-> B -(-2)-> E`.
/// The backpropagation from `E` meets a recursive cycle, so the network is not DC, and the
/// explanation relies on the maximum durations of both links.
#[test]
#[expect(clippy::many_single_char_names, reason = "the paper's point names")]
fn morris_2014_example_cycle_is_not_dc() {
    let mut stnu = Stnu::new();
    let [a] = points(&mut stnu);
    let b = stnu.add_contingent_link(a, Ticks(1), Ticks(3)).unwrap();
    let [c] = points(&mut stnu);
    let d = stnu.add_contingent_link(c, Ticks(1), Ticks(4)).unwrap();
    let [e] = points(&mut stnu);
    // E -4-> B and B -(-2)-> E: B - E in [2, 4]. B -1-> D and D -3-> B: D - B in [-3, 1].
    stnu.add_constraint(e, b, Some(Ticks(2)), Some(Ticks(4)))
        .unwrap();
    stnu.add_constraint(b, d, Some(Ticks(-3)), Some(Ticks(1)))
        .unwrap();

    assert!(!all_checkers_dc(&stnu));

    let Controllability::Uncontrollable(cycle) =
        stnu.check_controllability(&Budget::UNLIMITED).unwrap()
    else {
        panic!("not DC");
    };
    assert_cycle_closes(&stnu, &cycle);
    assert_eq!(cycle.contingent_links(), [ContingentId(0), ContingentId(1)]);
    assert!(
        cycle
            .edges()
            .any(|x| x == LabelledEdge::UpperCase(ContingentId(1)))
    );
}
