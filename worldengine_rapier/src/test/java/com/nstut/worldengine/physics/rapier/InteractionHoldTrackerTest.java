package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InteractionHoldTrackerTest {
    @Test void expiryRenewalAndRemovalMatchIndependentClockModel() {
        InteractionHoldTracker holds = new InteractionHoldTracker();
        java.util.Map<Integer, Long> model = new java.util.HashMap<>();
        java.util.Random random = new java.util.Random(471105);
        for (long tick = 0; tick < 1200; tick++) {
            int id = random.nextInt(32);
            if (tick % 11 == 0) { holds.remove(id); model.remove(id); }
            else { long expiry = tick + 1 + random.nextInt(40); holds.renew(id, expiry); model.put(id, expiry); }
            IntOpenHashSet actual = new IntOpenHashSet();
            java.util.Set<Integer> expected = new java.util.HashSet<>();
            var iterator = model.entrySet().iterator();
            while (iterator.hasNext()) {
                var entry = iterator.next();
                if (entry.getValue() <= tick) { expected.add(entry.getKey()); iterator.remove(); }
            }
            holds.drainExpired(tick, actual);
            assertEquals(expected, actual, "tick " + tick);
            // Renew just-expired bodies, as component reconciliation does, then
            // verify no duplicate deadline causes a later false expiry.
            for (int expired : expected) if ((expired & 1) == 0) {
                holds.renew(expired, tick + 20); model.put(expired, tick + 20);
            }
            for (int candidate = 0; candidate < 32; candidate++) {
                assertEquals(model.containsKey(candidate), holds.holds(candidate, tick), "tick " + tick + ", id " + candidate);
            }
        }
        holds.clear();
        assertTrue(holds.isEmpty());
        for (int id = 0; id < 32; id++) assertFalse(holds.holds(id, 1200));
    }
    @Test void repeatedRenewalExpiresAtLatestDeadline() {
        InteractionHoldTracker holds = new InteractionHoldTracker();
        IntOpenHashSet expired = new IntOpenHashSet();
        for (int tick = 0; tick < 120; tick++) {
            holds.renew(7, tick + 40);
            holds.drainExpired(tick, expired);
            assertTrue(expired.isEmpty());
            assertTrue(holds.holds(7, tick));
        }
        holds.drainExpired(158, expired);
        assertTrue(expired.isEmpty());
        holds.drainExpired(159, expired);
        assertEquals(Set.of(7), expired);
        assertFalse(holds.holds(7, 159));
        assertTrue(holds.isEmpty());
    }
    @Test void removedAndReusedIdDoesNotInheritStaleDeadline() {
        InteractionHoldTracker holds = new InteractionHoldTracker();
        IntOpenHashSet expired = new IntOpenHashSet();
        holds.renew(7, 40); holds.remove(7); holds.renew(7, 80);
        holds.renew(9, 60);
        holds.drainExpired(40, expired);
        assertTrue(expired.isEmpty());
        holds.drainExpired(60, expired);
        assertEquals(Set.of(9), expired);
        assertTrue(holds.holds(7, 60));
        holds.clear();
        assertTrue(holds.isEmpty());
        assertFalse(holds.holds(7, 60));
    }
}
