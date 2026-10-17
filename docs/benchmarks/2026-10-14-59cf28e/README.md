# Empty-dimension persistence: World Engine 0.1.3 vs stock Sable

Source `59cf28ed0ab029df03a0854533e7ac187960e45f`, measured on 2026-10-14. All 30 selected measured launches and four restart launches passed. One final launch had a port collision before measurement; the entire final pair was rerun in fresh worlds with identical frozen inputs. The rejected attempt and discarded baseline are retained in `failed-port-attempt.json` inside the evidence archive. The complete comparison is publication eligible under the documented protocol.

These measurements compare the complete World Engine addon with stock Sable 2.0.5. They do not isolate this persistence change from World Engine's existing physics optimizations. Idle tick improvements and mixed save results must be reported separately.

## Protocol

- Minecraft 1.21.1, NeoForge 21.1.228, Java 21, Windows, checked-in release native bundle, 2 GiB heap.
- Five alternating fresh-JVM pairs for each of 1, 16 and 64 extra datapack dimensions, in addition to the three vanilla dimensions. Every extra dimension was checked to have zero loaded chunks, structures and force-load tickets.
- Each process used 100 warmup ticks and 300 measured ticks, with 30 separately timed production `saveAllChunks` calls. Save logging was disabled in both variants.
- Explicit saves ran every ten ticks to collect samples. This is an accelerated save workload, not the normal autosave interval. Idle tick timing excludes these save calls; process CPU includes them.
- Frozen source/classes/resources/jars/native hashes were checked around every launch. Trial medians are independent observations; ticks are not pooled to inflate sample size. CPU uses full-window averages because Windows counters are coarse.

## Median results

Cells show stock Sable -> World Engine, followed by percentage change. Negative values mean lower cost; positive values mean higher cost.

| Metric | 1 extra dimension | 16 extra dimensions | 64 extra dimensions |
| --- | ---: | ---: | ---: |
| Idle tick ms | 1.199 -> 0.550 (-54.1%) | 1.760 -> 0.784 (-55.4%) | 4.146 -> 0.865 (-79.1%) |
| Idle p95 ms | 2.692 -> 2.083 (-22.6%) | 13.197 -> 2.214 (-83.2%) | 58.636 -> 7.740 (-86.8%) |
| Idle server-thread CPU ms/tick | 0.625 -> 0.469 (-25.0%) | 1.146 -> 0.469 (-59.1%) | 2.188 -> 0.781 (-64.3%) |
| Idle allocation bytes/tick | 27280.000 -> 25296.000 (-7.3%) | 103320.000 -> 89336.000 (-13.5%) | 199760.000 -> 186584.000 (-6.6%) |
| World save ms | 12.816 -> 14.891 (+16.2%) | 13.339 -> 15.419 (+15.6%) | 15.707 -> 18.886 (+20.2%) |
| Save p95 ms | 21.584 -> 29.089 (+34.8%) | 27.762 -> 26.910 (-3.1%) | 30.545 -> 112.719 (+269.0%) |
| Save server-thread CPU ms/save | 4.688 -> 6.771 (+44.4%) | 7.812 -> 8.333 (+6.7%) | 7.812 -> 7.812 (+0.0%) |
| Save allocation bytes/save | 129360.000 -> 119888.000 (-7.3%) | 324024.000 -> 280288.000 (-13.5%) | 924528.000 -> 768056.000 (-16.9%) |
| Process CPU ms/tick (accelerated saves included) | 4.740 -> 3.333 (-29.7%) | 15.469 -> 10.260 (-33.7%) | 35.990 -> 19.844 (-44.9%) |

## Trial spread

Q1-Q3 of the five trial medians; these are not confidence intervals.

| Extra dimensions | Metric | Stock Sable Q1-Q3 ms | World Engine Q1-Q3 ms |
| ---: | --- | ---: | ---: |
| 1 | Idle tick | 0.879-1.250 | 0.528-0.707 |
| 1 | World save | 10.717-13.117 | 13.684-15.899 |
| 16 | Idle tick | 1.704-1.842 | 0.747-0.861 |
| 16 | World save | 10.292-17.013 | 15.240-16.237 |
| 64 | Idle tick | 2.838-4.537 | 0.831-1.072 |
| 64 | World save | 9.745-19.779 | 17.111-20.234 |

## Persistence and validation

- Every stock-Sable extra dimension created an empty force-load ticket file; World Engine created zero across all 15 addon trials. These are file-presence counts after shutdown, not a syscall/write-count profile.
- Real holding sublevels were saved with zero active structures, then reloaded. Force-loaded structures survived a server restart. Removing the last structure and its ticket, followed by another restart, did not restore either.
- Storage probes checked write/flush/reopen, deletion, the public output-stream path, clean flush suppression, and pending state after failed writes and flushes.
- Both loader builds and 23 Python evidence tests passed. Both release jars contain the persistence mixins and exclude benchmark code.
- [Linux CI paired persistence and physics smoke](https://github.com/UpperMoon0/World-Engine/actions/runs/38589582123) passed after dependency-download HTTP 502 failures were retried. Successful physics artifact: `11906739896`; successful persistence artifact: `11907812750`. Earlier failed artifacts remain in the run and must not be mistaken for the successful retry.
- The broader required `cuboidWithNewCenterHoleFallsPastItsFormerSupport` GameTest fails locally and in Linux CI. It also fails locally with all three new persistence mixins disabled, using the unchanged main behavior. This inherited failure is retained in the compressed logs; the PR remains draft. No claim is made that the full GameTest suite is green.

## Limits

Whole-world saves include vanilla world/chunk serialization and filesystem I/O. Their measured wall-time changes are mixed; this report does not claim uniformly faster saves or an overall performance win. All measurements, including slower outcomes and tails, remain in `result.json` inside the evidence archive.

Previously used dimensions may still have Sable dirty holding chunks. This change conservatively preserves that state rather than clearing it through APIs that swallow write failures. Active/sleeping structures retain Sable serialization; this is not general dirty tracking for body data. Fabric builds, but Fabric runtime persistence, client FPS, retained RAM and arbitrary modpacks were not benchmarked.

## Evidence

- [Readable summary and archive SHA-256 manifest](summary.json)
- [Complete original raw samples, rejected attempt and 38 compressed logs](evidence.zip)
- [Frozen input SHA-256 manifest](inputs.json)
- [Empty ticket file counts](empty-ticket-files.json)

The archive replaces 40 individually tracked generated files without changing a
byte of their contents. In particular, the failed-attempt snapshot repeats 28
selected measurements and all four restart records; retaining that original
snapshot in compressed form keeps the audit trail without a second 58,555-line
JSON diff. `summary.json` includes SHA-256 digests for every archived file.
These are historical measurements of `59cf28e`, not measurements of subsequent
review fixes. Extract for inspection with:

```sh
python -m zipfile -e docs/benchmarks/2026-10-14-59cf28e/evidence.zip build/persistence-report
python -m unittest discover -s tools -p 'test_persistence_report.py'
```

New benchmark launches continue writing disposable JSON and logs under ignored
`build/persistence-comparison/`; CI retains those as workflow artifacts.
