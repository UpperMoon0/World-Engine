//! Phase 0 synthetic scaling matrix from plan.md.
//!
//! Run the complete matrix with:
//! `cargo bench -p worldengine_rapier --bench phase0_scaling`
//! Run a smoke pass with:
//! `cargo bench -p worldengine_rapier --bench phase0_scaling -- --quick`

use criterion::{BatchSize, BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use marten::octree::SubLevelOctree;
use rapier3d::math::Vec3;
use rapier3d::prelude::*;
use std::hint::black_box;
use worldengine_rapier::scene::{BodyAabb, WorldSpatialIndex};

struct BenchScene {
    pipeline: PhysicsPipeline,
    params: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
}

impl BenchScene {
    fn new() -> Self {
        Self {
            pipeline: PhysicsPipeline::new(),
            params: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
        }
    }

    fn add_body(
        &mut self,
        position: Vec3,
        velocity: Vec3,
        sleeping: bool,
        collider: bool,
    ) -> RigidBodyHandle {
        let handle = self.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(position)
                .linvel(velocity)
                .sleeping(sleeping)
                .build(),
        );
        if collider {
            self.colliders.insert_with_parent(
                ColliderBuilder::ball(0.55)
                    .active_events(ActiveEvents::COLLISION_EVENTS)
                    .build(),
                handle,
                &mut self.bodies,
            );
        }
        handle
    }

    fn step(&mut self) {
        self.pipeline.step(
            Vec3::ZERO,
            &self.params,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &(),
        );
    }

    fn counters(&self) -> SceneCounters {
        SceneCounters {
            rapier_bodies: self.bodies.len(),
            awake_bodies: self
                .bodies
                .iter()
                .filter(|(_, body)| !body.is_sleeping())
                .count(),
            active_bodies: self.islands.active_bodies().count(),
            colliders: self.colliders.len(),
            candidate_pairs: self.narrow_phase.contact_pairs().count(),
            contact_manifolds: self
                .narrow_phase
                .contact_pairs()
                .map(|pair| pair.manifolds.len())
                .sum(),
            ccd_bodies: self
                .bodies
                .iter()
                .filter(|(_, body)| body.is_ccd_enabled())
                .count(),
            joints: self.impulse_joints.len(),
        }
    }

    fn emit_profile(&self, scenario: &str) {
        let counts = self.counters();
        let rapier = &self.pipeline.counters;
        eprintln!(
            "PHASE0_PROFILE scenario={scenario} step_ms={:.6} broad_ms={:.6} narrow_ms={:.6} solver_ms={:.6} ccd_ms={:.6} bodies={} awake={} active={} colliders={} pairs={} manifolds={} ccd_bodies={} joints={}",
            rapier.step_time_ms(),
            rapier.broad_phase_time_ms(),
            rapier.narrow_phase_time_ms(),
            rapier.solver_time_ms(),
            rapier.ccd_time_ms(),
            counts.rapier_bodies,
            counts.awake_bodies,
            counts.active_bodies,
            counts.colliders,
            counts.candidate_pairs,
            counts.contact_manifolds,
            counts.ccd_bodies,
            counts.joints,
        );
    }
}

struct SceneCounters {
    rapier_bodies: usize,
    awake_bodies: usize,
    active_bodies: usize,
    colliders: usize,
    candidate_pairs: usize,
    contact_manifolds: usize,
    ccd_bodies: usize,
    joints: usize,
}

fn sleeping_scene(count: usize) -> BenchScene {
    let mut scene = BenchScene::new();
    for id in 0..count {
        scene.add_body(
            Vec3::new(id as Real * 4.0, 0.0, 0.0),
            Vec3::ZERO,
            true,
            false,
        );
    }
    scene.step();
    scene
}

fn isolated_scene(count: usize) -> BenchScene {
    let mut scene = BenchScene::new();
    for id in 0..count {
        let x = (id % 1_000) as Real * 4.0;
        let z = (id / 1_000) as Real * 4.0;
        scene.add_body(
            Vec3::new(x, 0.0, z),
            Vec3::new(1.0, 0.0, 0.25),
            false,
            false,
        );
    }
    scene.step();
    scene
}

fn clustered_scene(count: usize, clusters: usize) -> BenchScene {
    let mut scene = BenchScene::new();
    let per_cluster = count / clusters;
    for cluster in 0..clusters {
        let origin = cluster as Real * 128.0;
        for local in 0..per_cluster {
            let x = (local % 10) as Real * 1.05;
            let z = ((local / 10) % 10) as Real * 1.05;
            let y = (local / 100) as Real * 1.05;
            scene.add_body(Vec3::new(origin + x, y, z), Vec3::ZERO, false, true);
        }
    }
    scene.step();
    scene
}

fn colliding_scene(count: usize) -> BenchScene {
    let mut scene = BenchScene::new();
    let width = (count as f64).cbrt().ceil() as usize;
    for id in 0..count {
        let x = id % width;
        let y = (id / width) % width;
        let z = id / (width * width);
        scene.add_body(
            Vec3::new(x as Real * 0.9, y as Real * 0.9, z as Real * 0.9),
            Vec3::ZERO,
            false,
            true,
        );
    }
    scene.step();
    scene
}

fn docking_scene(count: usize) -> BenchScene {
    let mut scene = BenchScene::new();
    let mut previous = None;
    for id in 0..count {
        let handle = scene.add_body(
            Vec3::new(id as Real * 1.25, 0.0, 0.0),
            Vec3::ZERO,
            false,
            true,
        );
        if let Some(parent) = previous {
            scene
                .impulse_joints
                .insert(parent, handle, FixedJointBuilder::new().build(), true);
        }
        previous = Some(handle);
    }
    scene.step();
    scene
}

fn huge_voxel_ship() -> SubLevelOctree {
    let mut octree = SubLevelOctree::new(7);
    for x in (0..128).step_by(4) {
        for y in (0..128).step_by(4) {
            for z in (0..128).step_by(4) {
                octree.insert(x, y, z, 1);
            }
        }
    }
    octree
}

fn bench_sleeping(c: &mut Criterion) {
    let mut group = c.benchmark_group("A-D_sleeping_bodies");
    group.sample_size(10);
    for count in [100, 1_000, 10_000, 50_000] {
        let mut scene = sleeping_scene(count);
        scene.emit_profile(&format!("sleeping_{count}"));
        group.throughput(Throughput::Elements(count as u64));
        group.bench_with_input(BenchmarkId::from_parameter(count), &count, |b, _| {
            b.iter(|| scene.step());
        });
        black_box(scene.counters());
    }
    group.finish();
}

fn bench_moving_and_clusters(c: &mut Criterion) {
    let mut moving = c.benchmark_group("E-F_moving_isolated");
    moving.sample_size(10);
    for count in [1_000, 10_000] {
        let mut scene = isolated_scene(count);
        scene.emit_profile(&format!("moving_isolated_{count}"));
        moving.bench_with_input(BenchmarkId::from_parameter(count), &count, |b, _| {
            b.iter(|| scene.step())
        });
        black_box(scene.counters());
    }
    moving.finish();

    let mut clusters = c.benchmark_group("G-H_clustered_bodies");
    clusters.sample_size(10);
    for cluster_count in [100, 10] {
        let mut scene = clustered_scene(1_000, cluster_count);
        scene.emit_profile(&format!("clusters_{cluster_count}"));
        clusters.bench_with_input(
            BenchmarkId::new("1000_bodies", cluster_count),
            &cluster_count,
            |b, _| {
                b.iter(|| scene.step());
            },
        );
        black_box(scene.counters());
    }
    clusters.finish();
}

fn bench_active_contacts(c: &mut Criterion) {
    let mut group = c.benchmark_group("I-K_active_collisions");
    group.sample_size(10);
    for count in [100, 500, 1_000] {
        let mut scene = colliding_scene(count);
        scene.emit_profile(&format!("active_collisions_{count}"));
        group.bench_with_input(BenchmarkId::from_parameter(count), &count, |b, _| {
            b.iter(|| scene.step())
        });
        black_box(scene.counters());
    }
    group.finish();
}

fn bench_voxel_geometry(c: &mut Criterion) {
    let ship = huge_voxel_ship();
    c.bench_function("L_huge_voxel_ship/query", |b| {
        let mut coordinate = 0;
        b.iter(|| {
            coordinate = (coordinate + 17) & 127;
            black_box(ship.query(coordinate, coordinate, coordinate, 0));
        });
    });

    let ships: Vec<_> = (0..100).map(|_| huge_voxel_ship()).collect();
    c.bench_function("M_100_huge_voxel_ships/query_each", |b| {
        b.iter(|| {
            for (index, ship) in ships.iter().enumerate() {
                black_box(ship.query((index as i32 * 17) & 127, 64, 64, 0));
            }
        });
    });

    c.bench_function("N_rapid_block_editing/4096", |b| {
        b.iter_batched(
            || SubLevelOctree::new(7),
            |mut edited| {
                for index in 0..4096 {
                    let x = index & 15;
                    let z = (index >> 4) & 15;
                    let y = (index >> 8) & 15;
                    edited.insert(x, y, z, 1);
                }
                black_box(edited);
            },
            BatchSize::SmallInput,
        );
    });
}

fn bench_docking_and_coordinates(c: &mut Criterion) {
    let mut docking = docking_scene(1_000);
    docking.emit_profile("docking_joints_1000");
    c.bench_function("O_many_docking_joints/1000", |b| b.iter(|| docking.step()));
    black_box(docking.counters());

    let mut group = c.benchmark_group("P-R_extreme_coordinates");
    for coordinate in [1_000_000.0, 100_000_000.0, 10_000_000_000.0] {
        let mut index = WorldSpatialIndex::default();
        for id in 0..10_000 {
            let position = Vec3::new(coordinate + id as Real * 64.0, coordinate, coordinate);
            index.update(id, BodyAabb::around(position, Vec3::splat(2.0)));
        }
        let query = BodyAabb::around(Vec3::splat(coordinate), Vec3::splat(32.0));
        group.bench_with_input(
            BenchmarkId::from_parameter(coordinate as u64),
            &coordinate,
            |b, _| {
                b.iter(|| black_box(index.query(query, usize::MAX)));
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_sleeping,
    bench_moving_and_clusters,
    bench_active_contacts,
    bench_voxel_geometry,
    bench_docking_and_coordinates,
);
criterion_main!(benches);
