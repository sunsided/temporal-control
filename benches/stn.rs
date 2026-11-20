//! Benchmarks for the ground-side STN algorithms on a sparse network: the full minimal network
//! (Floyd-Warshall) against the constrained pairs only (P³C).

use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use temporal_control::{Budget, Stn, Ticks, TimePoint};

/// A ladder of `rungs` rungs: two chains of steps `[1, 3]` with a rung `[0, 2]` between the
/// two points of each rung. Sparse (`m` about `3n / 2`) with induced width 2, so P³C does
/// linear work where Floyd-Warshall does cubic.
fn ladder(rungs: usize) -> Stn {
    let mut stn = Stn::new();

    for _ in 0..2 * rungs {
        stn.add_time_point().unwrap();
    }

    let step = (Some(Ticks(1)), Some(Ticks(3)));

    for r in 0..rungs {
        let (top, bottom) = (TimePoint(2 * r), TimePoint(2 * r + 1));
        stn.add_constraint(top, bottom, Some(Ticks(0)), Some(Ticks(2)))
            .unwrap();

        if r + 1 < rungs {
            stn.add_constraint(top, TimePoint(2 * r + 2), step.0, step.1)
                .unwrap();
            stn.add_constraint(bottom, TimePoint(2 * r + 3), step.0, step.1)
                .unwrap();
        }
    }

    stn
}

/// Floyd-Warshall and P³C on ladders of 50 and 200 rungs.
fn benches(c: &mut Criterion) {
    for rungs in [50, 200] {
        let stn = ladder(rungs);
        c.bench_function(&format!("minimal_network_ladder_{rungs}"), |b| {
            b.iter(|| black_box(&stn).minimal_network(&Budget::UNLIMITED).unwrap());
        });
        c.bench_function(&format!("p3c_ladder_{rungs}"), |b| {
            b.iter(|| {
                black_box(&stn)
                    .partial_path_consistency(&Budget::UNLIMITED)
                    .unwrap()
            });
        });
    }
}

criterion_group!(group, benches);
criterion_main!(group);
