import copy
import tempfile
import unittest
from pathlib import Path

from benchmark_persistence import compare_pair, summarize, validate, write_dimensions


def sample(engine="sable"):
    point = dict(tick=0, wallMs=1.0, cpuMs=0.5, allocatedBytes=100)
    return dict(schema=1, engine=engine, scenario="dimensions1", runId="run", mode="measure",
                correctnessPass=True, storageChecksPass=True, extraDimensions=1, saveLogging=False,
                warmupTicks=10, measuredTicks=20, saveIntervalTicks=10,
                ticks=[dict(point, tick=i) for i in range(20)],
                saves=[dict(point, tick=i) for i in (0, 10)], processCpuMs=10,
                java="21", os="os", processors=4, maxHeapBytes=2000,
                mods=["sable:2.0.5"])


class PersistenceEvidenceTests(unittest.TestCase):
    def test_reject_missing_save_or_tick(self):
        for field in ("ticks", "saves"):
            row = sample()
            row[field].pop()
            with self.assertRaisesRegex(ValueError, "Incomplete"):
                validate(row, "sable", "dimensions1", "run", "measure", 10, 20)

    def test_reject_missing_dimension_and_failed_restart(self):
        row = sample()
        row["extraDimensions"] = 0
        with self.assertRaisesRegex(ValueError, "extraDimensions"):
            validate(row, "sable", "dimensions1", "run", "measure", 10, 20)
        row = dict(sample(), mode="restored", correctnessPass=False)
        with self.assertRaisesRegex(ValueError, "correctnessPass"):
            validate(row, "sable", "dimensions1", "run", "restored", 10, 20)

    def test_reject_changed_pair(self):
        for field in ("saveIntervalTicks", "extraDimensions", "java", "mods", "saveLogging"):
            base, addon = sample(), sample("worldengine")
            addon[field] = ["sable:other"] if field == "mods" else "different"
            with self.assertRaises(ValueError):
                compare_pair(base, addon)

    def test_trials_remain_independent_and_cpu_uses_window_mean(self):
        first, second = sample(), copy.deepcopy(sample())
        first["ticks"][0]["cpuMs"] = 10.5
        second["ticks"] = [dict(p, wallMs=3) for p in second["ticks"]]
        summary = summarize([first, second])[0]
        self.assertEqual(summary["ticks.wallMs"]["median"], 2)
        self.assertEqual(summary["ticks.cpuMs"]["median"], 0.75)

    def test_datapack_has_exact_dimension_count(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_dimensions(root, 16)
            self.assertEqual(len(list((root / "world/datapacks/we_persistence/data/we_persistence/dimension").glob("*.json"))), 16)


if __name__ == "__main__":
    unittest.main()
