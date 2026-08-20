use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use rapier3d::math::Vec3;
use rapier3d::prelude::Real;
use std::hint::black_box;
use worldengine_rapier::scene::{BodyAabb, MACRO_CELL_SIZE, WorldSpatialIndex};

fn build_sparse_index(count: usize) -> WorldSpatialIndex {
    let mut index = WorldSpatialIndex::default();
    for id in 0..count {
        let x = (id % 10_000) as Real * (MACRO_CELL_SIZE * 2.0);
        let z = (id / 10_000) as Real * (MACRO_CELL_SIZE * 2.0);
        index.update(id, BodyAabb::around(Vec3::new(x, 0.0, z), Vec3::splat(2.0)));
    }
    index
}

fn benchmark_sparse_queries(c: &mut Criterion) {
    let mut group = c.benchmark_group("sparse_world_query");
    group.sample_size(20);
    for count in [10_000, 100_000] {
        let index = build_sparse_index(count);
        let query = BodyAabb::around(Vec3::ZERO, Vec3::splat(16.0));
        group.bench_with_input(BenchmarkId::new("nearby_aabb", count), &count, |b, _| {
            b.iter(|| black_box(index.query(query, usize::MAX)));
        });
    }
    group.finish();
}

fn benchmark_swept_queries(c: &mut Criterion) {
    let index = build_sparse_index(100_000);
    let start = BodyAabb::around(Vec3::ZERO, Vec3::splat(2.0));
    c.bench_function("swept_world_query/100000", |b| {
        b.iter(|| {
            let swept = start.swept(Vec3::new(MACRO_CELL_SIZE * 3.0, 0.0, 0.0));
            black_box(index.query(swept, usize::MAX))
        });
    });
}

criterion_group!(benches, benchmark_sparse_queries, benchmark_swept_queries);
criterion_main!(benches);
