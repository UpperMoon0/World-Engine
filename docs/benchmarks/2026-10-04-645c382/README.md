# World Engine 0.1.1 vs stock Sable 2.0.5

This clean, unprofiled release-bundle comparison passed all paired physics checks.
It covers Minecraft 1.21.1 dedicated servers on NeoForge with five paired trials
per workload: 50 fresh JVMs, each with 300 warmup and 300 measured ticks.

- [CI comparison](https://github.com/UpperMoon0/World-Engine/actions/runs/37151439949)
- Measured source: [`645c382c4c67`](https://github.com/UpperMoon0/World-Engine/commit/645c382c4c679dba885f07cc81d8f471f0c9a927)
- Evidence ID: `1791059095-2311`
- [All raw measurements and runtime evidence](result.json.gz)
- [Frozen source, class, resource and binary hashes](inputs.json.gz)
- Native bundle SHA-256: `2299f1351d5946ee6a8fa60de958d583c53c6f77082cc146c9898d8f9dcaa401`

The figure measures the linked source revision. Later optimization commits need
a separate comparison; these archived numbers do not measure those commits.
The normal checked-in native bundle was used; development profiler libraries and
JFR recordings are excluded from these performance results.

## Results

Total process CPU and median/p95 tick latency improved across all five workloads.
Active-body server-thread CPU increased 13.4%, and terrain-edit Java allocation
increased 1.8%. This run does not meet the target of improving every measure.

Each cell gives stock Sable → World Engine and the percentage change. Lower is
better. Tick and allocation summaries are medians of each trial's median; CPU is
the median of each trial's full-window process CPU per tick. The p95 summary is
the median of the five per-trial 95th percentiles.

| Workload | Median tick (ms) | Process CPU (ms/tick) | Java allocation (KiB/tick) | p95 tick (ms) |
| --- | ---: | ---: | ---: | ---: |
| Idle | 0.547 → 0.292 (-46.7%) | 6.20 → 5.23 (-15.6%) | 22.5 → 22.5 (+0.0%) | 0.740 → 0.467 (-36.9%) |
| 64 supported bodies | 0.920 → 0.423 (-54.1%) | 6.73 → 5.87 (-12.9%) | 38.1 → 29.3 (-23.0%) | 1.190 → 0.619 (-48.0%) |
| 256 supported bodies | 1.882 → 0.713 (-62.1%) | 7.50 → 5.97 (-20.4%) | 122.8 → 63.3 (-48.5%) | 2.374 → 0.991 (-58.2%) |
| 64 active bodies | 2.224 → 2.115 (-4.9%) | 13.27 → 10.17 (-23.4%) | 41.2 → 39.5 (-4.2%) | 5.551 → 2.578 (-53.6%) |
| 64 bodies + terrain edits | 5.335 → 2.549 (-52.2%) | 19.03 → 12.27 (-35.6%) | 46.8 → 47.6 (+1.8%) | 5.977 → 3.235 (-45.9%) |

![World Engine 0.1.1 stock Sable comparison](world-engine-0.1.1.png)

Maximum ticks are also retained. These are medians of the five per-trial maxima,
not a pooled maximum or a promise about every individual tick.

| Workload | Sable maximum (ms) | World Engine maximum (ms) | Change |
| --- | ---: | ---: | ---: |
| Idle | 0.934 | 0.692 | -25.9% |
| 64 supported bodies | 1.491 | 0.833 | -44.1% |
| 256 supported bodies | 2.700 | 1.322 | -51.0% |
| 64 active bodies | 6.822 | 3.304 | -51.6% |
| 64 bodies + terrain edits | 7.729 | 6.055 | -21.7% |

Average server-thread CPU over the measured tick intervals is retained alongside
total process CPU:

| Workload | Server-thread CPU (ms/tick), Sable → World Engine |
| --- | ---: |
| Idle | 0.426 → 0.309 (-27.4%) |
| 64 supported bodies | 0.744 → 0.425 (-42.8%) |
| 256 supported bodies | 1.672 → 0.707 (-57.7%) |
| 64 active bodies | 0.975 → 1.105 (+13.4%) |
| 64 bodies + terrain edits | 1.052 → 1.053 (+0.2%) |

The raw summary includes each metric's minimum, maximum and quartiles across
trials. Quartiles describe spread, not confidence intervals; individual ticks
are correlated and are not counted as independent trials.

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

Process CPU includes native workers, networking, GC and workload commands between
tick intervals. Java allocation is the
server thread's cumulative allocation during measured tick intervals, not
retained memory, peak RAM or native allocation. Preparation, assembly, workload
commands and output I/O are outside those tick intervals. See the complete
[protocol and metric definitions](../../../BENCHMARKS.md).

## Validation and coverage

The implementation has 25 passing Java tests, 54 passing native tests, 13 passing
benchmark/JNI evidence tests. Of 13 runtime cases, all 10 required and two optional
cases pass; the optional assembly case fails in
[the measured revision's build](https://github.com/UpperMoon0/World-Engine/actions/runs/37151443163).
All six native release targets were rebuilt in
[native CI](https://github.com/UpperMoon0/World-Engine/actions/runs/37141492644).
Runtime regressions cover sleeping terrain support, support removal, a center
hole, rectangular-floor removal, half-height slabs and interacting bodies across
a forced-chunk region boundary. A ballistic exact-query assertion reproduces a
live-body query miss before bounding-box-driven refresh and passes after the fix.
The boundary test reproduces failure with the
preceding library and passes with the corrected bundle.

The optional all-blocks assembly test reports `create:item_drain` and
`create:millstone` with both World Engine and stock Sable. A separate local
Windows baseline run excluded the World Engine addon and used Sable's own
Rapier native library: both stock required cases passed, while the same two
block IDs failed assembly. Stock also failed its optional gravity and snag tests
in that run. The assembly source records these failures when entity counts change
after assembly; full compatibility for those cases remains unresolved. The
[stock runtime log](stock-gametests.log.gz) and
[baseline-only Gradle setup](stock-gametests.init.gradle) are archived. This
local correctness run supplies no performance numbers for the Linux figure.

The test counts follow the source annotations: eight World Engine cases and two
stock cases are required; stock gravity, snag and all-blocks assembly are optional.
Minecraft's log misleadingly prints the total number of cases as the number of
required tests. The stock run executes five cases, with two required passes and
three optional failures; the addon run executes 13, with 12 passes and one
optional failure.

These five server fixtures do not certify client FPS, observer networking,
save/reload, arbitrary large ships, joints, Fabric runtime or every addon.

## Reproduce

Use Java 21 and Python 3.11+ on an otherwise idle machine:

```sh
python tools/benchmark_sable.py --trials 5 --warmup 300 --ticks 300
```

Install matplotlib to recreate the figure from the archived measurements:

```sh
python tools/plot_benchmark.py docs/benchmarks/2026-10-04-645c382/result.json.gz docs/benchmarks/2026-10-04-645c382/world-engine-0.1.1.png --version 0.1.1
```

The stock GameTest baseline uses the archived setup and the same Java 21:

```sh
./gradlew -I docs/benchmarks/2026-10-04-645c382/stock-gametests.init.gradle -PbenchmarkEngine=sable :benchmark:runStockGameTest
```
