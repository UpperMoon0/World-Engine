"""Validate and combine full workload shards without mixing machines within pairs."""
import argparse
import json
from pathlib import Path
from benchmark_sable import SCENARIOS, compare_pair, summarize, validate


def merge(paths, run_id):
    shards, runs, manifests, hosts = {}, [], {}, {}
    source_head = None
    shared_inputs = None
    for path in paths:
        data = json.loads(path.read_text())
        if (data.get("schema") != 1 or data.get("pass_") is not True
                or data.get("sourceDirty") is not False or data.get("profiled") is not False
                or data.get("nativeMode") != "checked-in-release-bundle"):
            raise ValueError("Unclean, profiled, failed or non-release shard")
        scenarios = {r["scenario"] for r in data["runs"]}
        if len(scenarios) != 1 or not scenarios.issubset(SCENARIOS):
            raise ValueError("Each shard must contain one known workload")
        scenario = next(iter(scenarios))
        if scenario in shards:
            raise ValueError("Duplicate workload shard")
        if len(data["runs"]) != 10:
            raise ValueError("Expected five complete pairs per workload")
        if source_head is None:
            source_head = data["sourceHead"]
        elif source_head != data["sourceHead"]:
            raise ValueError("Mixed source revisions")
        manifest = json.loads((path.parent / "inputs.json").read_text())
        # Generated classes/JARs can carry runner-local build details; retain
        # every complete manifest, and require identical sources and release bytes.
        source_inputs = {k: v for k, v in manifest.items() if "/build/" not in k}
        bundle = "worldengine_rapier/src/main/resources/natives/worldengine_rapier/worldengine_rapier_binaries.zip.l4z"
        if bundle not in source_inputs:
            raise ValueError("Missing frozen release bundle")
        if shared_inputs is None:
            shared_inputs = source_inputs
        elif shared_inputs != source_inputs:
            raise ValueError("Mixed frozen source or release inputs")
        by_run = {r["runId"]: r for r in data["runs"]}
        expected_ids = {f"{scenario}-{trial}-{engine}"
                        for trial in range(5) for engine in ("sable", "worldengine")}
        if set(by_run) != expected_ids:
            raise ValueError("Missing, duplicate or mismatched trial IDs")
        reference_environment = None
        for trial in range(5):
            pair = {}
            for engine in ("sable", "worldengine"):
                sample = by_run[f"{scenario}-{trial}-{engine}"]
                if sample["warmupTicks"] != 300 or sample["measuredTicks"] != 300:
                    raise ValueError("Incomplete warmup or measurement window")
                validate(sample, engine, scenario, sample["runId"], 300)
                if engine == "worldengine" and "worldengine:0.1.1" not in sample["mods"]:
                    raise ValueError("Missing World Engine 0.1.1 runtime")
                environment = {k: sample[k] for k in
                    ("java", "vm", "processors", "os", "maxHeapBytes", "physicsConfig", "mods")}
                if engine == "sable":
                    if reference_environment is None:
                        reference_environment = environment
                    elif environment != reference_environment:
                        raise ValueError("Environment drift between trials in one workload")
                pair[engine] = sample
            compare_pair(pair["sable"], pair["worldengine"])
        shards[scenario] = data["runId"]
        hosts[scenario] = data["host"]
        manifests[scenario] = manifest
        runs.extend(data["runs"])
    if set(shards) != set(SCENARIOS):
        raise ValueError("Missing workload shard")
    result = dict(schema=1, pass_=True, publicationEligible=True, sourceHead=source_head,
        sourceDirty=False, profiled=False, nativeMode="checked-in-release-bundle",
        runId=run_id, host="Separate CI runner per workload; see shardHosts",
        shardHosts=hosts, shardRunIds=shards,
        protocol="Five paired trials on one runner per workload; workloads run in parallel",
        summary=summarize(runs), runs=runs)
    return result, shared_inputs, manifests


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    result, shared, manifests = merge(sorted(args.input.rglob("result.json")), args.run_id)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, data in (("result.json", result), ("inputs.json", shared), ("shard-inputs.json", manifests)):
        (args.output / name).write_text(json.dumps(data, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "runs"}, indent=2))


if __name__ == "__main__":
    main()
