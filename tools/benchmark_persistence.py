#!/usr/bin/env python3
"""Paired real-server dimension saves, idle ticks and restart/deletion checks."""
import argparse
import json
import math
import os
from pathlib import Path
import platform
import socket
import statistics
import subprocess
import time

from benchmark_sable import fingerprint, quantiles, run_command


def write_dimensions(game, count):
    pack = game / "world/datapacks/we_persistence"
    definitions = pack / "data/we_persistence/dimension"
    definitions.mkdir(parents=True)
    (pack / "pack.mcmeta").write_text(json.dumps({"pack": {
        "pack_format": 48, "description": "Disposable persistence benchmark dimensions"}}))
    definition = {"type": "minecraft:overworld", "generator": {"type": "minecraft:flat",
        "settings": {"biome": "minecraft:plains", "lakes": False, "features": False,
                     "layers": [{"block": "minecraft:bedrock", "height": 1}]}}}
    for index in range(count):
        (definitions / f"empty_{index}.json").write_text(json.dumps(definition))


def validate(row, engine, scenario, run_id, mode, warmup, ticks):
    for field, expected in {"schema": 1, "engine": engine, "scenario": scenario,
            "runId": run_id, "mode": mode, "correctnessPass": True,
            "extraDimensions": int(scenario[10:]), "warmupTicks": warmup,
            "measuredTicks": ticks, "saveIntervalTicks": 10, "saveLogging": False}.items():
        if row.get(field) != expected:
            raise ValueError(f"Invalid {field}: {row.get(field)}; {row.get('correctnessError')}")
    if mode != "measure":
        if row.get("restartChecksPass") is not True:
            raise ValueError("Missing restart check")
        return
    if row.get("storageChecksPass") is not True:
        raise ValueError("Missing storage regression checks")
    for field, expected in (("ticks", list(range(ticks))),
                            ("saves", list(range(0, ticks, 10)))):
        if [r["tick"] for r in row[field]] != expected:
            raise ValueError("Incomplete measurement window")
        for metric in ("wallMs", "cpuMs", "allocatedBytes"):
            quantiles([r[metric] for r in row[field]])
    quantiles([row["processCpuMs"]])


def compare_pair(base, addon):
    for field in ("scenario", "extraDimensions", "java", "os", "processors", "maxHeapBytes",
                  "warmupTicks", "measuredTicks", "saveIntervalTicks", "saveLogging"):
        if base[field] != addon[field]:
            raise ValueError(f"Unequal paired input: {field}")
    mods = {m for m in addon["mods"] if not m.startswith(("worldengine:", "worldengine_rapier:"))}
    if set(base["mods"]) != mods:
        raise ValueError("Unequal Sable/dependency versions")


def summarize(runs):
    result = []
    for count in sorted({r["extraDimensions"] for r in runs}):
        for engine in ("sable", "worldengine"):
            selected = [r for r in runs if r["extraDimensions"] == count and r["engine"] == engine]
            if not selected:
                continue
            row = dict(extraDimensions=count, engine=engine, trials=len(selected))
            for window in ("ticks", "saves"):
                for metric in ("wallMs", "cpuMs", "allocatedBytes"):
                    # CPU counters can have coarse resolution; use full-window means.
                    measure = statistics.mean if metric == "cpuMs" else statistics.median
                    row[f"{window}.{metric}"] = quantiles([measure([s[metric] for s in r[window]]) for r in selected])
                row[f"{window}.p95Ms"] = quantiles([sorted(s["wallMs"] for s in r[window])[
                    math.ceil(.95 * len(r[window])) - 1] for r in selected])
                row[f"{window}.maxMs"] = quantiles([max(s["wallMs"] for s in r[window]) for r in selected])
            row["processCpuMsPerTick"] = quantiles([r["processCpuMs"] / r["measuredTicks"] for r in selected])
            result.append(row)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--warmup", type=int, default=100)
    parser.add_argument("--ticks", type=int, default=300)
    parser.add_argument("--dimensions", type=int, nargs="+", default=[1, 16, 64])
    parser.add_argument("--timeout", type=int, default=600)
    args = parser.parse_args()
    if min(args.trials, args.warmup, args.ticks, args.timeout, *args.dimensions) <= 0 or args.warmup % 10 or args.ticks % 10:
        parser.error("Positive counts required; tick windows must be multiples of ten")
    root = Path(__file__).resolve().parents[1]
    stamp = time.strftime("%Y%m%d-%H%M%S")
    evidence = root / "build/persistence-comparison" / stamp
    evidence.mkdir(parents=True)
    lock = root / "build/sable-comparison.lock"
    fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    wrapper = str(root / ("gradlew.bat" if os.name == "nt" else "gradlew"))
    common = ["--console=plain", "--max-workers=2", "-Dorg.gradle.jvmargs=-Xmx1280m"]
    if os.name == "nt":
        for task in ("buildLocalRustWindows", "copyLocalRustWindows", "packRustNatives"):
            common += ["-x", f":worldengine_rapier:{task}"]
    env = os.environ.copy() | {"WE_BENCH_PROFILE": "false", "WE_BENCH_NATIVE_PROFILE_PATH": ""}
    runs, restarts = [], []
    metadata = dict(schema=1, host=platform.platform(), runId=stamp, pass_=False,
                    nativeMode="checked-in-release-bundle", trials=args.trials)
    try:
        metadata["sourceHead"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        metadata["sourceDirty"] = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip())
        env["WE_BENCH_GAME_DIR"] = str(evidence / "prepare")
        for engine in ("sable", "worldengine"):
            run_command([wrapper, f"-PbenchmarkEngine={engine}", ":benchmark:classes", ":neoforge:classes",
                         ":worldengine_rapier:jar", ":benchmark:prepareServerRun", *common],
                        root, env, evidence / f"prepare-{engine}.log", args.timeout)
        frozen = fingerprint(root)
        (evidence / "inputs.json").write_text(json.dumps(frozen, indent=2))
        for trial in range(args.trials):
            for count in args.dimensions:
                pair = {}
                for engine in (("sable", "worldengine") if trial % 2 == 0 else ("worldengine", "sable")):
                    name = f"dimensions{count}-{trial}-{engine}"
                    game = evidence / name
                    game.mkdir()
                    write_dimensions(game, count)
                    (game / "config").mkdir()
                    (game / "config/sable-common.toml").write_text(
                        "sub_level_saving_log_message=false\nverbose_serialization_logging=false\n")
                    with socket.socket() as sock:
                        sock.bind(("127.0.0.1", 0))
                        port = sock.getsockname()[1]
                    (game / "eula.txt").write_text("eula=true\n")
                    (game / "server.properties").write_text(f"server-ip=127.0.0.1\nserver-port={port}\nonline-mode=false\n"
                        "level-type=minecraft:flat\nlevel-seed=470101\ngenerate-structures=false\n"
                        'generator-settings={"layers":[{"block":"minecraft:bedrock","height":1},'
                        '{"block":"minecraft:dirt","height":2},{"block":"minecraft:grass_block","height":1}],'
                        '"biome":"minecraft:plains","features":false,"lakes":false}\n'
                        "view-distance=2\nsimulation-distance=2\nspawn-protection=0\nmax-players=1\n")
                    restart = trial == 0 and count == args.dimensions[0]
                    for mode in (("measure", "restored", "deleted") if restart else ("measure",)):
                        output = game / ("sample.json" if mode == "measure" else f"{mode}.json")
                        trial_env = env | dict(WE_BENCH_GAME_DIR=str(game), WE_BENCH_OUTPUT=str(output),
                            WE_BENCH_SCENARIO=f"dimensions{count}", WE_BENCH_RUN=name,
                            WE_BENCH_WARMUP=str(args.warmup), WE_BENCH_TICKS=str(args.ticks),
                            WE_PERSIST_MODE=mode, WE_PERSIST_RESTART_CHECK=str(restart).lower())
                        if fingerprint(root) != frozen:
                            raise RuntimeError("Frozen inputs changed before launch")
                        print(f"Starting {name} ({mode})", flush=True)
                        run_command([wrapper, f"-PbenchmarkEngine={engine}", ":benchmark:runServer", *common],
                                    root, trial_env, game / f"{mode}-launch.log", args.timeout)
                        row = json.loads(output.read_text())
                        validate(row, engine, f"dimensions{count}", name, mode, args.warmup, args.ticks)
                        if fingerprint(root) != frozen:
                            raise RuntimeError("Frozen inputs changed during launch")
                        if mode == "measure":
                            pair[engine] = row
                            runs.append(row)
                        else:
                            restarts.append(row)
                compare_pair(pair["sable"], pair["worldengine"])
        metadata["summary"] = summarize(runs)
        metadata["pass_"] = True
        metadata["publicationEligible"] = (not metadata["sourceDirty"] and args.trials >= 5
            and args.warmup >= 100 and args.ticks >= 300 and set(args.dimensions) == {1, 16, 64}
            and len(restarts) == 4)
    except Exception as exc:
        metadata["error"] = str(exc)
    finally:
        metadata["runs"], metadata["restartRuns"] = runs, restarts
        (evidence / "result.json").write_text(json.dumps(metadata, indent=2))
        os.close(fd)
        lock.unlink()
    print(json.dumps({k: v for k, v in metadata.items() if k not in ("runs", "restartRuns")}, indent=2), flush=True)
    print(f"Evidence: {evidence}", flush=True)
    return 0 if metadata["pass_"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
