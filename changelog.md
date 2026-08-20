# World Engine 1.0.0

- Migrated the project from a Sable fork to a standalone Sable addon.
- Preserved the world-scale Rapier registry, tiers, sparse index, local regions, merge/split logic, incremental voxel geometry, parallel scheduler, terrain streaming, and network LOD.
- Moved Sable hot-path changes to addon-owned mixins and extension interfaces.
- Namespaced Java classes, JNI exports, native resources, cache paths, metadata, artifacts, and CI workflows.
- Removed copied Sable implementation, compatibility layers, assets, wiki, scratch sources, and Sable publication configuration.
- Retained and fixed the terrain-footprint regression work and scheduler scaling benchmarks.
