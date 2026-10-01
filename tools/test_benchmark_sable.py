import unittest
from benchmark_sable import compare_pair, quantiles, summarize, validate


def sample(engine="sable"):
    config = dict(substepsPerTick=2, contactSpringFrequency=40, contactSpringDampingRatio=5,
                  solverIterations=18, pgsIterations=2, stabilizationIterations=2,
                  minDynamicBodiesPerIsland=128)
    settings = {k: v for k, v in config.items() if k != "substepsPerTick"}
    return dict(schema=1, engine=engine, scenario="idle", runId="run", correctnessPass=True,
                queryChecks=19, samples=[dict(tick=0, tickMs=1, serverThreadCpuMs=1,
                    serverThreadAllocatedBytes=100)], processCpuMs=1, bodies=0, finalPoses=[],
                blocksPerBody=4, physicsConfig=config, java="21", vm="vm",
                submittedSceneSettingsAtStart={"1": settings.copy()},
                submittedSceneSettingsAtEnd={"1": settings.copy()},
                processors=4, os="os", maxHeapBytes=2000, warmupTicks=100, measuredTicks=1,
                mods=["sable:2.0.5"])


class EvidenceTests(unittest.TestCase):
    def test_late_scene_defaults_rejected(self):
        row = sample("worldengine")
        row["submittedSceneSettingsAtEnd"]["2"] = dict(
                row["submittedSceneSettingsAtEnd"]["1"], solverIterations=4)
        with self.assertRaisesRegex(ValueError, "Unequal submitted solver settings"):
            validate(row, "worldengine", "idle", "run", 1)

    def test_missing_scene_configuration_rejected(self):
        row = sample("worldengine")
        del row["submittedSceneSettingsAtEnd"]
        with self.assertRaisesRegex(ValueError, "Missing submitted solver settings"):
            validate(row, "worldengine", "idle", "run", 1)

    def test_failed_physics_rejected(self):
        failed = sample("worldengine")
        failed["correctnessPass"] = False
        failed["correctnessError"] = "Body escaped terrain support"
        with self.assertRaisesRegex(ValueError, "Body escaped terrain support"):
            validate(failed, "worldengine", "idle", "run", 1)

    def test_missing_ticks_rejected(self):
        with self.assertRaises(ValueError):
            validate(sample(), "sable", "idle", "run", 2)

    def test_wrong_run_rejected(self):
        with self.assertRaises(ValueError):
            validate(sample(), "sable", "idle", "stale-run", 1)

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            quantiles([float("nan")])

    def test_coarse_cpu_counter_keeps_accumulated_work(self):
        row = sample()
        row["measuredTicks"] = 3
        row["samples"] = [
            dict(tick=i, tickMs=1, serverThreadCpuMs=cpu, serverThreadAllocatedBytes=100)
            for i, cpu in enumerate((0, 15.625, 0))
        ]
        summary = summarize([row])[0]
        self.assertEqual(summary["serverThreadCpuMs"]["median"], 0)
        self.assertAlmostEqual(summary["serverThreadCpuMsPerTick"]["median"], 15.625 / 3)

    def test_substep_reduction_rejected(self):
        left = sample()
        right = sample("worldengine")
        right["physicsConfig"]["substepsPerTick"] = 1
        with self.assertRaisesRegex(ValueError, "physicsConfig"):
            compare_pair(left, right)

    def test_dependency_drift_rejected(self):
        left = sample()
        right = sample("worldengine")
        right["mods"] = ["sable:2.0.6", "worldengine:0.1.0"]
        with self.assertRaises(ValueError):
            compare_pair(left, right)

    def test_correct_pairs_accepted(self):
        left = sample()
        right = sample("worldengine")
        right["mods"].append("worldengine:0.1.0")
        validate(left, "sable", "idle", "run", 1)
        compare_pair(left, right)


if __name__ == "__main__":
    unittest.main()
