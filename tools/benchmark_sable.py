#!/usr/bin/env python3
"""Paired, fresh-JVM dedicated-server comparisons. Never publishes partial runs."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import socket
import statistics
import subprocess
import time

SCENARIOS = ("idle", "supported64", "supported256", "active64", "edits64")


def quantiles(values):
    values = sorted(values)
    if not values or any(not math.isfinite(x) or x < 0 for x in values):
        raise ValueError("Invalid metric samples")
    return dict(median=statistics.median(values), min=min(values), max=max(values),
                q1=values[(len(values)-1)//4], q3=values[(3*(len(values)-1)+3)//4])


def validate(sample, engine, scenario, run_id, ticks):
    if sample.get("correctnessPass") is not True:
        raise ValueError("Physics correctness failed: " + sample.get("correctnessError", "missing evidence"))
    if (sample["schema"] != 1 or sample["engine"] != engine
            or sample["scenario"] != scenario or sample["runId"] != run_id
            or sample["correctnessPass"] is not True or sample["queryChecks"] != 19):
        raise ValueError("Missing/mismatched correctness evidence")
    rows = sample["samples"]
    if len(rows) != ticks or [r["tick"] for r in rows] != list(range(ticks)):
        raise ValueError("Incomplete tick evidence")
    for field in ("tickMs", "serverThreadCpuMs", "serverThreadAllocatedBytes"):
        quantiles([r[field] for r in rows])
    quantiles([sample["processCpuMs"]])
    if engine == "worldengine":
        for field in ("submittedSceneSettingsAtStart", "submittedSceneSettingsAtEnd"):
            scenes = sample.get(field)
            if not isinstance(scenes, dict) or not scenes:
                raise ValueError("Missing submitted solver settings")
            for settings in scenes.values():
                if not settings or any(sample["physicsConfig"].get(k) != v for k, v in settings.items()):
                    raise ValueError("Unequal submitted solver settings")
                if set(settings) != {"contactSpringFrequency", "contactSpringDampingRatio",
                        "solverIterations", "pgsIterations", "stabilizationIterations",
                        "minDynamicBodiesPerIsland"}:
                    raise ValueError("Incomplete submitted solver settings")
    expected = {"idle": 0, "supported64": 64, "supported256": 256, "active64": 64, "edits64": 64}[scenario]
    if sample["bodies"] != expected or len(sample["finalPoses"]) != expected:
        raise ValueError("Fixture population changed")
    for pose in sample["finalPoses"]:
        if any(not math.isfinite(pose[k]) for k in ("x", "y", "z")):
            raise ValueError("Non-finite body pose")


def fingerprint(root):
    paths = set()
    for pattern in ("*.gradle", "*.properties", "buildSrc/**", "tools/*.py",
                    "benchmark/src/**", "*/src/main/**", "*/build/classes/**",
                    "*/build/resources/main/**", "*/build/libs/*.jar"):
        paths.update(p for p in root.glob(pattern) if p.is_file() and "__pycache__" not in p.parts
                     and not (p.is_relative_to(root / "buildSrc" / "build")
                              or p.is_relative_to(root / "buildSrc" / ".gradle")
                              or p.is_relative_to(root / "worldengine_rapier" / "src" / "main" / "rust" / "target")))
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def run_command(command, root, env, log, timeout):
    with log.open("w", encoding="utf-8") as output:
        process = subprocess.Popen(command, cwd=root, env=env, stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=os.name != "nt")
        try:
            code = process.wait(timeout=timeout)
        except BaseException:
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                import signal
                os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise
    if code:
        raise RuntimeError(f"Command failed ({code}); inspect {log}")
    text = log.read_text(encoding="utf-8", errors="replace")
    if any(marker in text for marker in ("Encountered an unexpected exception", "Critical injection failure",
                                        "This crash report has been saved", "BUILD FAILED")):
        raise RuntimeError(f"Fatal output despite successful exit: {log}")


def compare_pair(base, addon):
    # Different providers/mod lists are intended; simulation and environment must match.
    for field in ("scenario", "bodies", "blocksPerBody", "physicsConfig", "java", "vm",
                  "processors", "os", "maxHeapBytes", "warmupTicks", "measuredTicks"):
        if base[field] != addon[field]:
            raise ValueError(f"Unequal paired input: {field}")
    base_mods = set(base["mods"])
    addon_mods = {m for m in addon["mods"] if not m.startswith(("worldengine:", "worldengine_rapier:"))}
    if base_mods != addon_mods:
        raise ValueError("Baseline and addon Sable/dependency versions differ")
    # Supported bodies must settle to equivalent poses; active bodies need the safety oracle,
    # not bitwise equality between two different native solvers.
    if base["scenario"].startswith("supported"):
        for left, right in zip(base["finalPoses"], addon["finalPoses"]):
            if any(abs(left[k] - right[k]) > 0.15 for k in ("x", "y", "z")):
                raise ValueError("Supported-body poses diverged beyond 0.15 blocks")


def summarize(runs):
    rows = []
    for scenario in SCENARIOS:
        for engine in ("sable", "worldengine"):
            selected = [r for r in runs if r["scenario"] == scenario and r["engine"] == engine]
            if not selected:
                continue
            row = dict(scenario=scenario, engine=engine, trials=len(selected))
            for metric in ("tickMs", "serverThreadCpuMs", "serverThreadAllocatedBytes"):
                # Trials are the independent unit. Do not pool ticks and invent sample size.
                row[metric] = quantiles([statistics.median([s[metric] for s in r["samples"]])
                                        for r in selected])
            row["p95TickMs"] = quantiles([sorted(s["tickMs"] for s in r["samples"])[
                math.ceil(.95 * len(r["samples"]))-1] for r in selected])
            row["maxTickMs"] = quantiles([max(s["tickMs"] for s in r["samples"]) for r in selected])
            # Windows thread CPU counters can advance in coarse quanta. The
            # per-tick median may be zero; also report CPU accumulated over the
            # full measured window, averaged per tick.
            row["serverThreadCpuMsPerTick"] = quantiles([
                sum(s["serverThreadCpuMs"] for s in r["samples"]) / r["measuredTicks"]
                for r in selected])
            row["processCpuMsPerTick"] = quantiles([r["processCpuMs"]/r["measuredTicks"] for r in selected])
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--warmup", type=int, default=100)
    parser.add_argument("--ticks", type=int, default=100)
    parser.add_argument("--timeout", type=int, default=600)
    parser.add_argument("--profile", action="store_true", help="Record JFR diagnostics; results are not publication eligible")
    parser.add_argument("--native-profile", type=Path,
                        help="Use a benchmark-profiler native library; implies --profile and freezes its hash")
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    args = parser.parse_args()
    if min(args.trials, args.warmup, args.ticks, args.timeout) <= 0:
        parser.error("All counts/timeouts must be positive")
    if args.native_profile:
        args.native_profile = args.native_profile.resolve()
        if not args.native_profile.is_file():
            parser.error("Native profiler library does not exist")
        args.profile = True
    root = Path(__file__).resolve().parents[1]
    stamp = f"{int(time.time())}-{os.getpid()}"
    evidence = root / "build" / "sable-comparison" / stamp
    evidence.mkdir(parents=True)
    lock = root / "build" / "sable-comparison.lock"
    fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    wrapper = str(root / ("gradlew.bat" if os.name == "nt" else "gradlew"))
    common = ["--no-daemon", "--console=plain", "--max-workers=2", "-Dorg.gradle.jvmargs=-Xmx1280m"]
    if os.name == "nt":
        # Measure the checked-in release bundle, also used by Linux release builds.
        # Windows' normal jar task rebuilds/repacks on every invocation, which
        # both requires a Rust developer environment and breaks frozen inputs.
        for task in ("buildLocalRustWindows", "copyLocalRustWindows", "packRustNatives"):
            common.extend(["-x", f":worldengine_rapier:{task}"])
    env = os.environ.copy()
    env["WE_BENCH_PROFILE"] = "true" if args.profile else "false"
    env["WE_BENCH_NATIVE_PROFILE_PATH"] = str(args.native_profile) if args.native_profile else ""
    env["WE_NATIVE_PROFILE"] = "true" if args.native_profile else "false"
    metadata = dict(schema=1, host=platform.platform(), runId=stamp, pass_=False,
                    nativeMode="development-profiler-override" if args.native_profile else "checked-in-release-bundle",
                    profiled=args.profile)
    def capture_inputs():
        captured = fingerprint(root)
        if args.native_profile:
            captured["developmentNative:" + str(args.native_profile)] = hashlib.sha256(args.native_profile.read_bytes()).hexdigest()
        return captured
    runs = []
    try:
        metadata["sourceHead"] = subprocess.check_output(
            ["git", "-c", f"safe.directory={root}", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        metadata["sourceDirty"] = bool(subprocess.check_output(
            ["git", "-c", f"safe.directory={root}", "status", "--porcelain"], cwd=root, text=True).strip())
        # Finish compilation before timing; prepare both variants.
        env["WE_BENCH_GAME_DIR"] = str(evidence / "prepare")
        for engine in ("sable", "worldengine"):
            run_command([wrapper, f"-PbenchmarkEngine={engine}", ":benchmark:classes",
                         ":neoforge:classes", ":worldengine_rapier:jar",
                         ":benchmark:prepareServerRun", *common],
                        root, env, evidence / f"prepare-{engine}.log", args.timeout)
        frozen = capture_inputs()
        (evidence / "inputs.json").write_text(json.dumps(frozen, indent=2))
        for trial in range(args.trials):
            for scenario in args.scenarios:
                pair = {}
                order = ("sable", "worldengine") if trial % 2 == 0 else ("worldengine", "sable")
                for engine in order:
                    name = f"{scenario}-{trial}-{engine}"
                    game = evidence / name
                    game.mkdir()
                    with socket.socket() as sock:
                        sock.bind(("127.0.0.1", 0))
                        port = sock.getsockname()[1]
                    (game / "eula.txt").write_text("eula=true\n")
                    (game / "server.properties").write_text(
                        f"server-ip=127.0.0.1\nserver-port={port}\nonline-mode=false\n"
                        "level-type=minecraft:flat\nlevel-seed=470101\n"
                        'generator-settings={"layers":[{"block":"minecraft:bedrock","height":1},'
                        '{"block":"minecraft:dirt","height":2},{"block":"minecraft:grass_block","height":1}],'
                        '"biome":"minecraft:plains","features":false,"lakes":false}\n'
                        "generate-structures=false\nview-distance=4\nsimulation-distance=4\n"
                        "spawn-protection=0\nmax-players=1\n")
                    output = game / "sample.json"
                    trial_env = env | dict(WE_BENCH_GAME_DIR=str(game), WE_BENCH_OUTPUT=str(output),
                        WE_BENCH_SCENARIO=scenario, WE_BENCH_RUN=name,
                        WE_BENCH_WARMUP=str(args.warmup), WE_BENCH_TICKS=str(args.ticks))
                    if capture_inputs() != frozen:
                        raise RuntimeError("Frozen source/runtime changed before trial")
                    print(f"Starting {name}", flush=True)
                    run_command([wrapper, f"-PbenchmarkEngine={engine}", ":benchmark:runServer", *common],
                                root, trial_env, game / "launch.log", args.timeout)
                    sample = json.loads(output.read_text())
                    validate(sample, engine, scenario, name, args.ticks)
                    if capture_inputs() != frozen:
                        raise RuntimeError("Frozen source/runtime changed during trial")
                    pair[engine] = sample
                    runs.append(sample)
                compare_pair(pair["sable"], pair["worldengine"])
        metadata["summary"] = summarize(runs)
        metadata["pass_"] = True
        metadata["publicationEligible"] = (not args.profile and not metadata["sourceDirty"] and args.trials >= 5 and args.warmup >= 100
                                           and args.ticks >= 100 and set(args.scenarios) == set(SCENARIOS))
    except Exception as exc:
        metadata["error"] = str(exc)
    finally:
        metadata["runs"] = runs
        (evidence / "result.json").write_text(json.dumps(metadata, indent=2))
        os.close(fd)
        lock.unlink()
    print(json.dumps({k: v for k, v in metadata.items() if k != "runs"}, indent=2), flush=True)
    print(f"Evidence: {evidence}", flush=True)
    return 0 if metadata["pass_"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
