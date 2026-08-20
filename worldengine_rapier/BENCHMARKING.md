# World-engine physics benchmarks

The Phase 0 benchmark foundation lives in `src/main/rust/rapier/benches/phase0_scaling.rs`.
It mirrors workloads A–R in `plan.md` and is intentionally separate from correctness tests.

The scaling acceptance matrix lives in `src/main/rust/rapier/benches/registry_scaling.rs`.
It constructs the production `PhysicsScene`, `SableSceneData`, `UniverseBody` registry,
`WorldSpatialIndex`, and tier scheduler. It holds the working set at 1,000 resident / 100
awake while varying real persistent records through 10,000, 100,000, and 1,000,000, then
runs the production Rapier step and active-pose export working set.

## Running it

From `worldengine_rapier/src/main/rust`:

```text
cargo bench -p worldengine_rapier --features benchmark-profiler --bench phase0_scaling
```

For a fast compilation/runtime smoke pass:

```text
cargo bench -p worldengine_rapier --features benchmark-profiler --bench phase0_scaling -- --quick
```

Run the fixed-active-set acceptance matrix with:

```text
cargo bench -p worldengine_rapier --bench registry_scaling -- --quick
```

Criterion accepts a final filter, so an individual workload can be isolated, for example:

```text
cargo bench -p worldengine_rapier --features benchmark-profiler --bench phase0_scaling -- --quick A-D_sleeping_bodies/1000
```

Reports are written under `target/criterion`. Keep results from the same machine, power plan,
JVM, native profile, and mod revision together; otherwise comparisons are misleading.

## Workload matrix

- A–D: 100, 1,000, 10,000, and 50,000 sleeping bodies.
- E–F: 1,000 and 10,000 isolated moving bodies.
- G–H: 1,000 bodies divided into 100 or 10 contact clusters.
- I–K: 100, 500, and 1,000 actively colliding bodies.
- L–M: one and 100 sparse 128³ voxel ships.
- N: a 4,096-block incremental edit batch.
- O: a 1,000-body fixed-joint assembly.
- P–R: spatial queries around 1 million, 100 million, and 10 billion blocks.

Every Rapier scene prints one `PHASE0_PROFILE` line before sampling. It contains total step,
broadphase, narrowphase, solver, and CCD times plus the number of Rapier, awake, truly active,
collider, candidate-pair, manifold, CCD-body, and joint entries. `active` comes from
`IslandManager::active_bodies()` and must not be replaced with `RigidBodySet::len()`.

## Full-game profiling

The native suite deliberately excludes Minecraft/JVM noise. Use the dedicated GameTest server
with Java Flight Recorder for total MSPT, Java prephysics/actor time, allocations, and memory.
Use the `PHASE0_PROFILE` counters for native stage attribution, and compare command-buffer bytes
with `14 + sum(command sizes)` and pose-export bytes with `60 × active poses`. The batched bridge
should perform one command call and one pose-export call per active region/substep.

For native allocations, run the same filtered benchmark under DHAT, heaptrack, or the platform
allocator profiler. Criterion's wall-clock results and Rapier stage counters remain the baseline;
allocation instrumentation should not be mixed into timing comparisons.

## Fixed-active-set baseline (2026-08-10, integration-backed)

The corrected quick matrix on the Windows development machine produced:

- 10,000 persistent: 1.463 ms midpoint
- 100,000 persistent: 4.600 ms midpoint
- 1,000,000 persistent: 3.754 ms midpoint

Quick mode has a very small sample and should not be used for fine comparisons, but this confirms
that the real million-entry registry/index can execute the fixed 1,000-resident / 100-awake path.
The earlier microsecond figures were invalid because that benchmark stepped a bare Rapier scene
beside an unrelated vector. Dedicated-server testing remains required for the JVM, Minecraft chunk
system, and mod integration layers.
