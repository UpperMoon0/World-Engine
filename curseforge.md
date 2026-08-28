# World Engine

World Engine is a performance addon for **Sable** on Minecraft 1.21.1. It targets large moving structures and physics-heavy worlds while preserving Sable's configured simulation quality.

## Why use it?

- Adaptive spatial queries avoid thousands of unnecessary hash lookups.
- Terrain-scale structures use exact bounding-box checks without enormous section indexes.
- Persistent native scheduling and incremental collision geometry reduce repeated work.
- Active-body synchronization, bounded parallel stepping, and network LOD improve scalability.
- Sable's configured physics substeps and exact collision semantics remain intact.

World Engine is a standalone addon, not a Sable fork. Install the correct Fabric or NeoForge jar alongside **Sable 2.0.5 or newer**.

## Requirements

- Minecraft 1.21.1
- Fabric or NeoForge
- Java 21
- Sable 2.0.5+

Source, documentation, and issue reports: https://github.com/UpperMoon0/World-Engine
