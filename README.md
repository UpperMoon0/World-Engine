# World Engine

World Engine is a standalone optimization addon for [Sable](https://github.com/ryanhcode/sable) on Minecraft 1.21.1. Sable remains a required dependency and provides sublevels, compatibility, rendering, and its public API. World Engine contributes a higher-priority physics provider, world-scale native scheduling, and narrowly scoped mixins for the Sable hot paths described in [plan.md](plan.md).

World Engine is not a Sable fork. Its Java classes use the `com.nstut.worldengine` namespace, its native library and cache are independently named, and its loader metadata declares Sable as a required dependency.

## Implemented optimizations

- Changed-body JNI input/output batching and active-pose synchronization
- Persistent native body registry with dormant, ballistic, active, and critical tiers
- Sparse swept-AABB world index and local-origin Rapier regions
- Incremental region merge/split maintenance and bounded parallel stepping
- Persistent section-based voxel geometry and incremental block edits
- Region-local terrain streaming with per-body footprint/refcount tracking
- Selective CCD, one normal physics step, collision aggregation, and work budgets
- Active-body Java scheduling, incremental section-index queries, and empty-container fast paths
- Distance-based network LOD with velocity-based client extrapolation
- Per-tick time, weather, and natural-spawn memoization

The exact migration coverage is recorded in [MIGRATION.md](MIGRATION.md).

## Building

Use Java 21, then run:

```powershell
.\gradlew.bat build
```

The regular Windows build recompiles and packages the local optimized native. Cross-platform release natives require Docker:

```powershell
.\gradlew.bat worldengine_rapier:buildImages
.\gradlew.bat worldengine_rapier:buildRustNatives
.\gradlew.bat build
```

Output jars are produced under `fabric/build/libs` and `neoforge/build/libs`. Install the matching Sable 2.0.4-or-newer jar alongside World Engine.
