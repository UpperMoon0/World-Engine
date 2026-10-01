# Stock Sable comparison

World Engine's native Criterion and Java JMH benchmarks measure individual hot paths.
They do **not** establish an in-game speedup over stock Sable. The dedicated-server
comparison below runs actual Sable 2.0.5 in fresh Minecraft 1.21.1 NeoForge JVMs,
first without the addon, then with the production addon/provider. No performance
improvement is claimed until the complete comparison passes.

## Current finding — comparison blocked by physics regression

On 1 October 2026, the stock Sable supported64 fixture passed twice on on-prem-1.
The addon fixture failed twice using the checked-in release native bundle and
production Java sources from main at `ac50f1643ac66df036c383822c11e2c89c9eebf4`.
The second run used an explicit identical flat-world generator and captured the
failed body's pose:

| Engine | First body's initial Y | Final Y after 200 ticks | Correctness |
| --- | --- | --- | --- |
| Stock Sable 2.0.5 | -55.5 | -57.50255 | Passed; settled on the stone support |
| World Engine 0.1.0 | -55.5 | -849.5 | Failed; escaped the support |

This is a dedicated server with forced loaded fixture chunks and no connected
players. Both runs use two physics substeps and the same solver settings.
The benchmark intentionally refuses to summarize this failed pair as a speedup.
Native-source/bundle consistency and terrain/residency handling still need
investigation before attributing the regression to a specific code path.

The [failed-run evidence](docs/benchmarks/stock-sable-failure-2026-10-01.json)
contains the raw ticks, body poses, environment, provider identity and frozen
source/runtime input hashes. The measured checkout contains benchmark-only
instrumentation and is recorded as dirty; it is not a packaged-release
certification. Original complete logs are retained on the test host.

## Run

Use Java 21 and Python 3.11+, on an otherwise idle machine:

The comparison uses the checked-in release native bundle, with its hash in the
input manifest. On Windows it excludes the automatic native rebuild/repack tasks,
so no Rust developer environment is needed and native inputs remain frozen.
This validates the bundled binary with current Java sources; it does not certify
that rebuilding the native Rust sources would produce an identical binary.

```sh
python tools/benchmark_sable.py
```

For harness validation only:

```sh
python tools/benchmark_sable.py --trials 1 --warmup 100 --ticks 100 --scenarios idle supported64
python -m unittest discover -s tools -p 'test_benchmark_sable.py'
```

The benchmark project is opt-in through `-PbenchmarkEngine`; it is absent from normal
builds and release jars. Baseline classpaths contain neither World Engine sources
nor its native provider. The harness checks both the loaded mod set and selected
provider, and rejects World Engine mixin resources in the baseline.

## Protocol

- Five independent paired trials per fixture. Engine order alternates by trial.
- Each engine starts a fresh JVM and disposable, identically seeded loopback-only
  flat world; no existing player world or public server is used.
- Same Sable version, loader, dependencies, Java runtime, heap size and physics
  configuration, including substeps. No reduced solver quality or forced GC.
- 100 warmup ticks, then 100 measured ticks per process. Preparation, assembly,
  workload commands, correctness checks and output I/O are outside tick intervals.
- Inputs are compiled before measurement and source/classes/resources/JAR hashes
  are checked around each trial. Changed inputs invalidate the comparison.
- Every trial must finish without a crash, produce all raw ticks, retain all bodies
  with finite supported poses and valid mass, and pass 19 independent exact-AABB
  query checks. Supported-body paired poses must agree within 0.15 blocks.
- All raw samples and launch logs are under `build/sable-comparison/<run-id>/`.
  Failed or abbreviated runs cannot be marked publication eligible.

| Fixture | Workload |
| --- | --- |
| idle | No physics bodies; addon overhead control |
| supported64 | 64 four-block bodies settling on solid terrain |
| supported256 | 256 four-block bodies settling on solid terrain |
| active64 | 64 supported bodies given identical periodic velocity increments |
| edits64 | 64 supported bodies with periodic nearby terrain edits |

## Metrics and interpretation

- **Tick ms:** elapsed time from the benchmark's server Pre hook to its Post hook.
  This includes production Minecraft and physics work between those hooks, not
  the server's 50 ms pacing delay. It is not engine-only time.
- **p95 and maximum tick ms:** per-trial tail tick latency.
- **Server-thread CPU ms and allocated bytes:** Java thread counters over those
  hook intervals. Allocations are cumulative, not retained or peak memory.
- **Process CPU ms per tick:** whole Minecraft JVM CPU over the measured window,
  including native workers, networking, GC and instrumentation. It is not CPU
  percentage. It prevents native worker offloading from appearing as free work.

Summaries use each trial's median (or p95/max), then report median, q1 and q3 across
trials. Individual ticks are correlated and are not treated as independent trials.
Quartiles show spread, not confidence intervals. Coarse OS CPU counters can hide
small differences; interpret them over the complete measurement window.

This initial suite is **server-only and NeoForge-only**. It does not certify client
FPS, networking with observers, persistence/reload, large voxel ships, contact
clusters, joints, million-body registries or Fabric. Those need additional paired
fixtures before making claims about them. Active-body runs check physical safety;
they do not claim bitwise trajectory equality between different native pipelines.

The design follows Perfomant Boom's real-runtime baseline, alternating order,
raw evidence and fail-closed correctness gates. Its graphical-client and
save/reload checks cover explosion-specific behavior and are not implied here.
