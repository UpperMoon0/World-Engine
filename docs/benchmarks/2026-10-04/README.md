# World Engine 0.1.1 vs stock Sable 2.0.5

This clean, unprofiled release-bundle comparison passed all paired physics checks.
It covers Minecraft 1.21.1 dedicated servers on NeoForge with five paired trials
per workload: 50 fresh JVMs, each with 300 warmup and 300 measured ticks.

- [CI comparison](https://github.com/UpperMoon0/World-Engine/actions/runs/37146595075)
- Measured source: [`7c3f1ffda477`](https://github.com/UpperMoon0/World-Engine/commit/7c3f1ffda477d9c32c1073970eb1bb2476810d0b)
- Evidence ID: `1791054298-2357`
- [All raw measurements and runtime evidence](result.json.gz)
- [Frozen source, class, resource and binary hashes](inputs.json.gz)
- Native bundle SHA-256: `2299f1351d5946ee6a8fa60de958d583c53c6f77082cc146c9898d8f9dcaa401`

The figure measures the linked source revision. Later optimization commits need
a separate comparison; these archived numbers do not measure those commits.
The normal checked-in native bundle was used; development profiler libraries and
JFR recordings are excluded from these performance results.

## Results

Each cell gives stock Sable â†’ World Engine and the percentage change. Lower is
better. Tick and allocation summaries are medians of each trial's median; CPU is
the median of each trial's full-window process CPU per tick. The p95 summary is
the median of the five per-trial 95th percentiles.

| Workload | Median tick (ms) | Process CPU (ms/tick) | Java allocation (KiB/tick) | p95 tick (ms) |
| --- | ---: | ---: | ---: | ---: |
| Idle | 0.660 â†’ 0.383 (-42.0%) | 6.23 â†’ 5.57 (-10.7%) | 22.5 â†’ 22.5 (+0.0%) | 0.798 â†’ 0.542 (-32.0%) |
| 64 supported bodies | 1.009 â†’ 0.443 (-56.1%) | 6.90 â†’ 5.90 (-14.5%) | 38.0 â†’ 29.3 (-22.7%) | 1.172 â†’ 0.615 (-47.5%) |
| 256 supported bodies | 2.357 â†’ 0.780 (-66.9%) | 7.90 â†’ 6.03 (-23.6%) | 110.8 â†’ 63.3 (-42.8%) | 2.623 â†’ 1.341 (-48.9%) |
| 64 active bodies | 2.317 â†’ 2.305 (-0.5%) | 13.50 â†’ 10.37 (-23.2%) | 41.2 â†’ 40.4 (-1.9%) | 5.676 â†’ 2.758 (-51.4%) |
| 64 bodies + terrain edits | 5.487 â†’ 2.852 (-48.0%) | 19.90 â†’ 12.27 (-38.4%) | 49.1 â†’ 47.6 (-3.1%) | 6.425 â†’ 3.443 (-46.4%) |

![World Engine 0.1.1 stock Sable comparison](world-engine-0.1.1.png)

Maximum ticks are also retained. These are medians of the five per-trial maxima,
not a pooled maximum or a promise about every individual tick.

| Workload | Sable maximum (ms) | World Engine maximum (ms) | Change |
| --- | ---: | ---: | ---: |
| Idle | 1.523 | 0.843 | -44.7% |
| 64 supported bodies | 1.437 | 0.884 | -38.5% |
| 256 supported bodies | 4.448 | 2.002 | -55.0% |
| 64 active bodies | 6.863 | 3.579 | -47.9% |
| 64 bodies + terrain edits | 8.301 | 6.497 | -21.7% |

The window-averaged server-thread CPU is a diagnostic alongside total process CPU:

| Workload | Server-thread CPU (ms/tick), Sable â†’ World Engine |
| --- | ---: |
| Idle | 0.536 â†’ 0.398 (-25.8%) |
| 64 supported bodies | 0.813 â†’ 0.441 (-45.8%) |
| 256 supported bodies | 2.141 â†’ 0.829 (-61.3%) |
| 64 active bodies | 1.112 â†’ 1.332 (+19.9%) |
| 64 bodies + terrain edits | 1.221 â†’ 1.313 (+7.6%) |

The raw summary includes each metric's minimum, maximum and quartiles across
trials. Quartiles describe spread, not confidence intervals; individual ticks
are correlated and are not counted as independent trials.

The active-body median tick result is effectively tied (0.5% lower). Its
server-thread CPU increases by 19.9%; terrain-edit server-thread CPU increases
by 7.6%, despite lower total process CPU. This is not an all-metrics improvement.
One World Engine active-body trial contains a 48.431 ms tick; the largest stock
Sable active-body tick is 7.500 ms. The median-of-maxima table does not hide or
exclude that individual outlier, which remains in the raw measurements.

## Inputs and correctness

- Runner: `Linux-6.17.0-1022-azure-x86_64-with-glibc2.39`; 4 reported processors.
- Java: `21.0.12.1+1-LTS`; VM: `OpenJDK 64-Bit Server VM`.
- Maximum heap: 2.0 GiB in both variants.
- Physics settings, identical in each pair: `solverIterations`=18, `pgsIterations`=2, `stabilizationIterations`=2, `contactSpringDampingRatio`=5.0, `contactSpringFrequency`=40.0, `minDynamicBodiesPerIsland`=128, `substepsPerTick`=2.
- Engine order alternates by trial. Every process uses a fresh, identically seeded
  flat world with no observers or existing player saves.
- Every trial retains the expected body population, finite supported poses and
  valid mass, and passes 19 independent exact-AABB queries. Supported paired
  poses agree within 0.15 blocks. Loaded mod sets and selected providers are checked.
- The input manifest is checked before and after each trial. Sources, classes,
  resources and shipped native/JAR inputs remain frozen throughout the comparison.

Process CPU includes native workers, networking and GC. Java allocation is the
server thread's cumulative allocation during measured tick intervals, not
retained memory, peak RAM or native allocation. Preparation, assembly, workload
commands and output I/O are outside those tick intervals. See the complete
[protocol and metric definitions](../../../BENCHMARKS.md).

## Validation and coverage

The implementation has 24 passing Java tests, 54 passing native tests, 13 passing
benchmark/JNI evidence tests, and all 13 required runtime tests pass in
[the measured revision's build](https://github.com/UpperMoon0/World-Engine/actions/runs/37146443357).
All six native release targets were rebuilt in
[native CI](https://github.com/UpperMoon0/World-Engine/actions/runs/37141492644).
Runtime regressions cover sleeping terrain support, support removal, a center
hole, rectangular-floor removal, half-height slabs and interacting bodies across
a forced-chunk region boundary. The boundary test reproduces failure with the
preceding library and passes with the corrected bundle.

The existing optional all-blocks assembly test still reports `create:item_drain`
and `create:millstone`. Its source records these failures when the entity count
changes after assembly; full compatibility for these cases remains unresolved.

These five server fixtures do not certify client FPS, observer networking,
save/reload, arbitrary large ships, joints, Fabric runtime or every addon.

## Reproduce

Use Java 21 and Python 3.11+ on an otherwise idle machine:

```sh
python tools/benchmark_sable.py --trials 5 --warmup 300 --ticks 300
```

Install matplotlib to recreate the figure from the archived measurements:

```sh
python tools/plot_benchmark.py docs/benchmarks/2026-10-04/result.json.gz docs/benchmarks/2026-10-04/world-engine-0.1.1.png --version 0.1.1
```
