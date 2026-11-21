//! Benchmarks for the onboard dispatcher (step, loading an encoding) and the ground-side DC
//! checks.

use std::hint::black_box;

use criterion::Criterion;
use temporal_control::{
    Budget, DcAlgorithm, DispatchAlgorithm, DispatchableNetwork, Dispatcher, Stnu, Ticks, TimePoint,
};

/// A chain of `links` activities with uncertain durations `[2, 6]`, each started at most 3 ticks
/// after the previous one ended, under an overall deadline that leaves room for every outcome.
fn chain(links: usize) -> Stnu {
    let mut stnu = Stnu::new();
    let origin = stnu.add_time_point().unwrap();
    let mut end = origin;

    for _ in 0..links {
        let start = stnu.add_time_point().unwrap();
        stnu.add_constraint(end, start, Some(Ticks(0)), Some(Ticks(3)))
            .unwrap();
        end = stnu.add_contingent_link(start, Ticks(2), Ticks(6)).unwrap();
    }

    let deadline = i64::try_from(links).unwrap() * 9;
    stnu.add_constraint(origin, end, Some(Ticks(0)), Some(Ticks(deadline)))
        .unwrap();
    stnu
}

/// The dispatchable network of [`chain`].
fn network(links: usize) -> DispatchableNetwork {
    match chain(links).dispatchable(&Budget::UNLIMITED).unwrap() {
        temporal_control::Controllability::Controllable(net) => net,
        temporal_control::Controllability::Uncontrollable(_) => panic!("chain is DC"),
    }
}

/// Runs a full ASAP dispatch with every contingent duration at its maximum.
fn dispatch_run(net: &DispatchableNetwork) -> Ticks {
    let mut d: Dispatcher<32, 2048> = net.dispatcher().unwrap();
    let mut out = [TimePoint(0); 32];
    let mut due = [None; 32];
    let mut observed = Vec::with_capacity(32);

    for t in 0..1000 {
        observed.clear();
        observed.extend(
            (0..net.points().len())
                .filter(|&p| due[p] == Some(t))
                .map(TimePoint),
        );
        let k = d.step(Ticks(t), &observed, &mut out).unwrap();

        for (p, kind) in net.points().iter().enumerate() {
            if let temporal_control::PointKind::Contingent { activation, hi, .. } = kind {
                let activated = out[..k].contains(activation) || observed.contains(activation);

                if activated {
                    due[p] = Some(t + hi.0);
                }
            }
        }

        if d.is_complete() {
            return Ticks(t);
        }
    }

    panic!("dispatch did not complete");
}

/// Registers the benchmarks.
fn benches(c: &mut Criterion) {
    let net = network(10);
    c.bench_function("dispatch_chain_10", |b| {
        b.iter(|| dispatch_run(black_box(&net)));
    });

    let bytes = net.encode().unwrap();
    let mut target = Box::new(Dispatcher::<32, 2048>::EMPTY);
    c.bench_function("load_encoded_chain_10", |b| {
        b.iter(|| target.load_encoded(black_box(&bytes)).unwrap());
    });

    let stnu = chain(10);

    for (name, algorithm) in [
        ("estnu_minimal_chain_10", DispatchAlgorithm::Minimal),
        ("estnu_morris2014_chain_10", DispatchAlgorithm::Morris2014),
        ("closure_chain_10", DispatchAlgorithm::Closure),
    ] {
        c.bench_function(name, |b| {
            b.iter(|| {
                black_box(&stnu)
                    .dispatchable_with(algorithm, &Budget::UNLIMITED)
                    .unwrap()
            });
        });
    }

    for (name, algorithm) in [
        ("morris2014_chain_10", DcAlgorithm::Morris2014),
        ("rul_minus_chain_10", DcAlgorithm::RulMinus),
        ("rul2021_chain_10", DcAlgorithm::Rul2021),
        ("per_label_backprop_chain_10", DcAlgorithm::PerLabelBackprop),
    ] {
        c.bench_function(name, |b| {
            b.iter(|| {
                black_box(&stnu)
                    .check_controllability_with(algorithm, &Budget::UNLIMITED)
                    .unwrap()
            });
        });
    }
}

/// Runs every benchmark in this file with criterion's command-line flags.
///
/// Written out instead of `criterion_main!`, whose generated functions carry no docs.
fn main() {
    let mut criterion = Criterion::default().configure_from_args();

    benches(&mut criterion);
    criterion.final_summary();
}
