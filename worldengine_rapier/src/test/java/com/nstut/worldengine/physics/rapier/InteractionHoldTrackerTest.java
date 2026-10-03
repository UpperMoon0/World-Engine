package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InteractionHoldTrackerTest {
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
