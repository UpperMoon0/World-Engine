import copy
import unittest
from benchmark_sable import compare_pair, quantiles, validate


def sample(engine="sable"):
    return dict(schema=1, engine=engine, scenario="idle", runId="run", correctnessPass=True,
                queryChecks=19, samples=[dict(tick=0, tickMs=1, serverThreadCpuMs=1,
                    serverThreadAllocatedBytes=100)], processCpuMs=1, bodies=0, finalPoses=[],
                blocksPerBody=4, physicsConfig={"substepsPerTick": 2}, java="21", vm="vm",
                processors=4, os="os", maxHeapBytes=2000, warmupTicks=100, measuredTicks=1,
                mods=["sable:2.0.5"])


class EvidenceTests(unittest.TestCase):
    def test_missing_ticks_rejected(self):
        with self.assertRaises(ValueError):
            validate(sample(), "sable", "idle", "run", 2)

    def test_wrong_run_rejected(self):
        with self.assertRaises(ValueError):
            validate(sample(), "sable", "idle", "stale-run", 1)

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            quantiles([float("nan")])

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
