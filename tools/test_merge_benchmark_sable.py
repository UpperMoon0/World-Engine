import copy
import gzip
import json
from pathlib import Path
import tempfile
import unittest
from benchmark_sable import SCENARIOS, summarize
from merge_benchmark_sable import merge


class ShardEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(__file__).resolve().parents[1] / "docs/benchmarks/2026-10-04"
        self.data = json.loads(gzip.decompress((root / "result.json.gz").read_bytes()))
        self.manifest = json.loads(gzip.decompress((root / "inputs.json.gz").read_bytes()))
        self.paths = []
        for scenario in SCENARIOS:
            folder = Path(self.directory.name) / scenario
            folder.mkdir()
            shard = copy.deepcopy(self.data)
            shard["runs"] = [r for r in shard["runs"] if r["scenario"] == scenario]
            shard["publicationEligible"] = False
            path = folder / "result.json"
            path.write_text(json.dumps(shard))
            (folder / "inputs.json").write_text(json.dumps(self.manifest))
            self.paths.append(path)

    def mutate(self, change):
        path = self.paths[0]
        data = json.loads(path.read_text())
        change(data)
        path.write_text(json.dumps(data))

    def test_complete_shards_preserve_raw_runs_and_trial_statistics(self):
        data, shared, manifests = merge(self.paths, "ci-test")
        self.assertTrue(data["publicationEligible"])
        self.assertEqual(50, len(data["runs"]))
        self.assertEqual(summarize(self.data["runs"]), data["summary"])
        self.assertEqual(set(SCENARIOS), set(manifests))
        self.assertEqual(set(SCENARIOS), set(data["shardHosts"]))
        self.assertEqual(self.manifest, manifests["idle"])
        self.assertTrue(shared)

    def test_missing_or_duplicate_workload_rejected(self):
        for paths in (self.paths[:-1], self.paths + [self.paths[0]]):
            with self.assertRaisesRegex(ValueError, "[Mm]issing|Duplicate"):
                merge(paths, "ci-test")

    def test_failed_dirty_profiled_or_non_release_shard_rejected(self):
        for field, value in (("pass_", False), ("sourceDirty", True), ("profiled", True),
                             ("nativeMode", "development-profiler-override")):
            original = self.paths[0].read_text()
            self.mutate(lambda d: d.update({field: value}))
            with self.assertRaisesRegex(ValueError, "Unclean"):
                merge(self.paths, "ci-test")
            self.paths[0].write_text(original)

    def test_mixed_revisions_and_release_inputs_rejected(self):
        original = self.paths[0].read_text()
        self.mutate(lambda d: d.update(sourceHead="different"))
        with self.assertRaisesRegex(ValueError, "Mixed source"):
            merge(self.paths, "ci-test")
        self.paths[0].write_text(original)
        manifest = dict(self.manifest)
        manifest["gradle.properties"] = "different"
        (self.paths[0].parent / "inputs.json").write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "Mixed frozen"):
            merge(self.paths, "ci-test")

    def test_duplicate_trials_and_invalid_physics_rejected(self):
        original = self.paths[0].read_text()
        self.mutate(lambda d: d["runs"].__setitem__(0, d["runs"][1]))
        with self.assertRaisesRegex(ValueError, "trial IDs"):
            merge(self.paths, "ci-test")
        self.paths[0].write_text(original)
        self.mutate(lambda d: d["runs"][0].update(correctnessPass=False))
        with self.assertRaisesRegex(ValueError, "Physics correctness"):
            merge(self.paths, "ci-test")


if __name__ == "__main__":
    unittest.main()
