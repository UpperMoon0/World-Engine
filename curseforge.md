# World Engine

World Engine is a performance addon for [Sable](https://www.curseforge.com/minecraft/mc-mods/sable) on Minecraft 1.21.1. It is designed for physics-heavy worlds and large moving structures, such as ships, stations, and terrain-scale contraptions.

## Why use it?

- **Less repeated work:** reuses unchanged terrain and collision data so busy worlds spend less time rebuilding it.
- **More efficient large structures:** handles huge moving builds without creating enormous spatial indexes, while keeping exact collision checks.
- **Lower physics overhead:** reduces the work needed to track and update physics bodies, especially as their number grows.
- **More efficient synchronization:** reduces unnecessary position updates between the server and clients.
- **Preserved simulation quality:** keeps Sable's configured physics substeps and solver settings.

## Benchmark results

In a comparison of **World Engine 0.1.1 against stock Sable 2.0.5**, total process CPU usage fell **11–40%** and median server tick time fell **23–63%** across five tested scenarios. The 95th-percentile server tick time (p95) fell **30–60%**.

![World Engine 0.1.1 benchmark against stock Sable](https://raw.githubusercontent.com/UpperMoon0/World-Engine/main/docs/benchmarks/2026-10-04-c38bf71/world-engine-0.1.1.png)

The tests covered an idle world, 64 and 256 supported physics bodies, 64 active bodies, and 64 bodies with terrain edits. Each scenario used five paired trials on fresh Minecraft 1.21.1 NeoForge dedicated servers, with 300 warmup ticks and 300 measured ticks per run.

These results measure version **0.1.1**. Performance varies with your world and hardware; the tests do not measure client FPS, total RAM usage, or every ship and addon combination.

See the [full results and raw measurements](https://github.com/UpperMoon0/World-Engine/blob/main/docs/benchmarks/2026-10-04-c38bf71/README.md) and [benchmark method](https://github.com/UpperMoon0/World-Engine/blob/main/BENCHMARKS.md).

## Requirements

- Minecraft 1.21.1
- Fabric or NeoForge
- Java 21
- [Sable 2.0.5 or newer](https://www.curseforge.com/minecraft/mc-mods/sable)

## Installation

1. Install Sable and its required dependencies for your Minecraft version and loader.
2. Download the World Engine jar for **Fabric or NeoForge**, matching your installation.
3. Place it in your `mods` folder alongside Sable.

Install only the World Engine variant for your loader.

## Compatibility

World Engine is a standalone addon. Sable continues to provide moving worlds, rendering, configuration, and compatibility integrations, while World Engine optimizes the physics work behind them.

Large structures retain exact collision and bounding-box checks. World Engine does not skip simulation steps to achieve its performance improvements.

## Updates and support

- [Version changelogs](https://github.com/UpperMoon0/World-Engine/tree/main/changelog)
- [Source and documentation](https://github.com/UpperMoon0/World-Engine)
- [Report an issue](https://github.com/UpperMoon0/World-Engine/issues)

When reporting a problem, include your Minecraft version, loader, Sable and World Engine versions, and relevant logs or crash reports.

World Engine is independently maintained by **NsTut**. Sable is developed by **RyanHCode and its contributors**.
