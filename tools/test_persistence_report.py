"""Keep the compact historical report verifiable without launching a server."""
import hashlib
import json
from pathlib import Path
import unittest
import zipfile

from benchmark_persistence import compare_pair, summarize, validate


REPORT = Path(__file__).resolve().parents[1] / "docs/benchmarks/2026-10-14-59cf28e"


class PersistenceReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.summary = json.loads((REPORT / "summary.json").read_text())
        cls.archive_bytes = (REPORT / cls.summary["evidenceArchive"]).read_bytes()
        with zipfile.ZipFile(REPORT / cls.summary["evidenceArchive"]) as archive:
            cls.files = {name: archive.read(name) for name in archive.namelist()}
        cls.result = json.loads(cls.files["result.json"])

    def test_archive_preserves_every_original_file(self):
        self.assertEqual(hashlib.sha256(self.archive_bytes).hexdigest(),
                         self.summary["evidenceArchiveSha256"])
        self.assertEqual({name: hashlib.sha256(data).hexdigest() for name, data in self.files.items()},
                         self.summary["archivedFiles"])
        self.assertEqual(len(self.files), 40)
        self.assertEqual(sum(name.endswith(".log.gz") for name in self.files), 38)

    def test_raw_samples_reproduce_readable_summary(self):
        runs = self.result["runs"]
        restarts = self.result["restartRuns"]
        self.assertEqual(len(runs), self.summary["measurementCount"])
        self.assertEqual(len(restarts), self.summary["restartCount"])
        self.assertEqual((len(runs), len(restarts)), (30, 4))
        for row in runs + restarts:
            validate(row, row["engine"], row["scenario"], row["runId"], row["mode"], 100, 300)
        for base, addon in zip(runs[::2], runs[1::2]):
            if base["engine"] != "sable":
                base, addon = addon, base
            compare_pair(base, addon)
        self.assertEqual(summarize(runs), self.result["summary"])
        for key, value in self.result.items():
            if key not in ("runs", "restartRuns"):
                self.assertEqual(self.summary[key], value, key)

    def test_rejected_attempt_is_retained_without_selecting_its_baseline(self):
        failed = json.loads(self.files["failed-port-attempt.json"])
        self.assertFalse(failed["pass_"])
        self.assertEqual(sum(row in self.result["runs"] for row in failed["runs"]), 28)
        self.assertEqual(failed["restartRuns"], self.result["restartRuns"])
        excluded = self.result["excludedAttempts"][0]
        selected_ids = {row["runId"] for row in self.result["runs"]}
        self.assertNotIn(excluded["discardedBaseline"], selected_ids)
        self.assertNotIn(excluded["failedLaunch"], selected_ids)


if __name__ == "__main__":
    unittest.main()
