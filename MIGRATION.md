# Sable addon migration matrix

This matrix is the release gate for the fork-to-addon conversion. “Migrated” means the behavior exists in addon-owned code and no modified Sable class is packaged by World Engine.

| Plan work | Addon mechanism | Status |
|---|---|---|
| Phase 0 benchmark matrix and profiler correction | Native Criterion benches and `worldengine_rapier/BENCHMARKING.md` | Migrated |
| O(N) spatial queries | Incremental addon-owned section/body index plus native sparse index | Migrated |
| Array-map and small-structure scaling | Optimized provider uses primitive hash maps/sets | Migrated |
| Selective CCD and one normal step | Native per-body tier/CCD logic plus active-set scheduler mixin | Migrated |
| Batched Java/native commands | World Engine JNI direct buffers | Migrated |
| Changed/active poses only | `WorldEnginePoseSynchronizer` bridge and physics-system mixin | Migrated |
| Persistent body registry | Native universe records separate from resident Rapier bodies | Migrated |
| Simulation tiers | Dormant, ballistic, active, and critical native scheduling | Migrated |
| Sparse global spatial index | `RapierWorldSpatialIndex` | Migrated |
| Local physics regions and origins | `RapierPhysicsRegion` and native scene rebasing | Migrated |
| Incremental merge/split graph | Dirty-cell/edge/component maintenance and native scene merge | Migrated |
| Persistent incremental voxel geometry | Section octrees and growable top-level hierarchy | Migrated |
| Region scheduling/parallelism | Active/dirty/due queues and bounded worker pool; callback regions serialize | Migrated |
| Fixed-assembly preparation | Stable native fixed-joint components and roots | Migrated as implemented; physical one-body aggregation remains future work in the original plan |
| Terrain collision streaming | Reverse section index, refcounts, swept footprints, and footprint cache | Migrated |
| Network/activity LOD | Distance-throttled changed-body sends and five-tick velocity extrapolation mixins | Migrated |
| Block-change direct ownership | Plot lookup redirect and O(1) native body-addressed edits | Migrated |
| Active Java actor/force/mass/ticket work | Injected active/continuous/next-active sets | Migrated |
| Empty-world overhead | Empty server-container fast path | Migrated |
| Time/weather/spawn hot paths | Addon-owned memoization mixins, with level identity included in the spawn cache | Migrated |
| Terrain support regression | Addon NeoForge GameTest and `TerrainFootprintTrackerTest` | Migrated |

## Architecture cleanup checks

- [x] Root project and artifacts use the World Engine identity.
- [x] Sable is resolved as an external loader-specific dependency.
- [x] No `dev.ryanhcode.sable` class is defined by addon Java sources.
- [x] The optimized provider is selected through Sable's documented service-provider seam.
- [x] Provider classes, JNI symbols, native files, native cache, and nested library mod ID are independently namespaced.
- [x] The release native bundle contains World Engine-namespaced x86-64 and ARM64 libraries for Linux, macOS, and Windows; local rebuilds preserve the other targets.
- [x] Fork-only platform implementations, compatibility mixins, assets, wiki, extracted Minecraft sources, and scratch rewrite scripts are removed.
- [x] Sable publishing destinations and project IDs are removed.
- [x] The user's uncommitted terrain-footprint fix and regression tests are retained.
- [x] Clean Fabric and NeoForge artifacts contain no Sable class definitions; NeoForge required GameTests and Fabric server/client startup have been exercised.
