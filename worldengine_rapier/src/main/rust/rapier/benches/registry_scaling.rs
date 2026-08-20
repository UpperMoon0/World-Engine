use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use worldengine_rapier::RegistryScalingHarness;

fn benchmark_persistent_registry_independence(c: &mut Criterion) {
    let mut group = c.benchmark_group("persistent_registry_independence");
    group.sample_size(20);

    for persistent_count in [10_000, 100_000, 1_000_000] {
        let scene =
            std::cell::RefCell::new(RegistryScalingHarness::new(persistent_count, 1_000, 100));

        group.bench_with_input(
            BenchmarkId::new("1000_resident_100_awake", persistent_count),
            &persistent_count,
            |bencher, _| {
                bencher.iter_batched(
                    || {
                        scene.borrow_mut().reset_scheduler_state(1_000);
                    },
                    |_| {
                        black_box(scene.borrow_mut().tick_and_export_poses());
                    },
                    criterion::BatchSize::SmallInput,
                );
            },
        );
    }
    group.finish();
}

fn benchmark_scheduler_behavior(c: &mut Criterion) {
    let mut group = c.benchmark_group("scheduler_behavior");
    group.sample_size(20);

    {
        let scene = std::cell::RefCell::new(RegistryScalingHarness::new(1_000_000, 1_000, 100));
        group.bench_function("1M_persistent_100k_ballistic_1k_due", |bencher| {
            bencher.iter_batched(
                || {
                    let mut scene = scene.borrow_mut();
                    scene.reset_scheduler_state(1_000);
                    scene.set_ballistic(1_000, 100_000);
                    scene.schedule_bodies_range(1_000, 1_000, 1);
                },
                |_| {
                    black_box(scene.borrow_mut().tick_and_export_poses());
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }

    {
        let scene = std::cell::RefCell::new(RegistryScalingHarness::new(100_000, 1_000, 100));
        group.bench_function("100k_scheduled_0_due", |bencher| {
            bencher.iter_batched(
                || {
                    let mut scene = scene.borrow_mut();
                    scene.reset_scheduler_state(1_000);
                    scene.schedule_bodies(100_000, 1_000);
                },
                |_| {
                    black_box(scene.borrow_mut().tick_and_export_poses());
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }

    {
        let scene = std::cell::RefCell::new(RegistryScalingHarness::new(100_000, 1_000, 100));
        group.bench_function("100k_scheduled_10k_due", |bencher| {
            bencher.iter_batched(
                || {
                    let mut scene = scene.borrow_mut();
                    scene.reset_scheduler_state(1_000);
                    scene.schedule_bodies(100_000, 100_000);
                    scene.schedule_bodies(10_000, 1);
                },
                |_| {
                    black_box(scene.borrow_mut().tick_and_export_poses());
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    benchmark_persistent_registry_independence,
    benchmark_scheduler_behavior
);
criterion_main!(benches);
