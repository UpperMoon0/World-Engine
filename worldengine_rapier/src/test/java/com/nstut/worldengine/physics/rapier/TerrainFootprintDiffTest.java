package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.longs.LongSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class TerrainFootprintDiffTest {
    @Test
    void reusedDesiredBufferAndReferenceCountsMatchIndependentAllOwnersOracle() {
        Random random = new Random(917201);
        LongSet[] stored = new LongSet[32];
        Map<Integer, Set<Long>> expectedOwners = new HashMap<>();
        Map<Long, Integer> references = new HashMap<>();
        LongSet desired = new LongOpenHashSet();
        for (int update = 0; update < 1200; update++) {
            int owner = random.nextInt(stored.length);
            if (stored[owner] == null) stored[owner] = new LongOpenHashSet();
            desired.clear();
            Set<Long> expected = new HashSet<>();
            for (int i = random.nextInt(20); i > 0; i--) {
                long section = random.nextInt(50) - 25L;
                desired.add(section);
                expected.add(section);
            }
            TerrainFootprintDiff.update(stored[owner], desired,
                    key -> references.compute(key, (ignored, count) -> count == 1 ? null : count - 1),
                    key -> references.merge(key, 1, Integer::sum));
            expectedOwners.put(owner, expected);
            // Mutating the shared buffer after the update must not change an owner.
            desired.clear(); desired.add(999_999L);
            Map<Long, Integer> oracle = new HashMap<>();
            expectedOwners.forEach((id, sections) -> {
                assertEquals(sections, new HashSet<>(stored[id]));
                sections.forEach(key -> oracle.merge(key, 1, Integer::sum));
            });
            assertEquals(oracle, references);
        }
    }
}
