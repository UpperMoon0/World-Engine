# Stock Sable comparison

World Engine's native Criterion and Java JMH benchmarks measure individual hot paths.
They do **not** establish an in-game speedup over stock Sable. The dedicated-server
comparison below runs actual Sable 2.0.5 in fresh Minecraft 1.21.1 NeoForge JVMs,
first without the addon, then with the production addon/provider. No performance
improvement is claimed until the complete comparison passes.

## Terrain support and native bundle correction

The first comparison exposed a real regression: stock Sable settled at
Y=-57.50255, while World Engine fell to Y=-849.5 after 200 ticks. The
[original failed evidence](docs/benchmarks/stock-sable-failure-2026-10-01.json)
is retained.

Tracing the Java ticket manager and native collision path found three problems:

- The ticket mixin renewed terrain tickets only for active bodies. Sleeping
  resident bodies lost their support after ticket expiry. Tickets now cover
  active and resident bodies without scanning the entire abstract registry.
- The universe scheduler counted solver substeps as 50 ms server ticks.
  A separate server-tick counter keeps ballistic elapsed time independent of
  the number of solver substeps.
- Configuration updates affected existing scenes only. Later body regions kept
  native defaults. The pipeline now copies the latest settings and applies them
  to every new region before simulation, including regions created after reload.
  Timings captured before this correction were discarded.

GameTests also exposed a stale x86-64 Windows DLL in the shipped bundle. Commit
`db3511e` replaced the CI-built DLL from `da52395` with an older local DLL.
The restored bundle uses the CI-built Windows entry. Its other five native
entries are byte-for-byte unchanged, and the complete Rust source tree is
unchanged since that CI build. A clean source rebuild and the restored bundle
both pass the required GameTests; the stale bundle failed terrain support.

The support GameTest now runs for 160 ticks, past native sleep and ticket
expiry. A ballistic GameTest checks elapsed time in open space; restoring the
old clock makes it fail. The benchmark also runs supported bodies for 200 ticks
and compares their final poses with stock Sable.

Performance results are specific to the fixtures, host, bundled natives and
loader tested. They are not a blanket performance claim.

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
- Addon evidence records the six solver settings submitted through all three JNI
  setters for every live scene at start and end. Missing or unequal submissions
  invalidate the run. These diagnostics verify successful setter calls, not
  native readback; the JNI setter mappings were checked against the Rust source.
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
  Window-averaged thread CPU is also reported because coarse Windows counters
  can give a zero per-tick median despite substantial accumulated work.
- **Process CPU ms per tick:** whole Minecraft JVM CPU over the measured window,
  including native workers, networking, GC and instrumentation. It is not CPU
  percentage. It prevents native worker offloading from appearing as free work.

Summaries use each trial's median (or p95/max, or full-window CPU average),
then report median, q1 and q3 across
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
