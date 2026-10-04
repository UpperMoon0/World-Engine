package com.nstut.worldengine.physics.rapier;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ResidentBodyTrackerTest {
    @Test void sleepingResidentsAndExtraActiveBodiesReceiveTicketsWithoutDuplicates() {
        ResidentBodyTracker<Object> tracker = new ResidentBodyTracker<>();
        Object sleeping = new Object(), active = new Object();
        tracker.add(sleeping);
        assertEquals(List.of(sleeping), tracker.tickets(List.of()));
        assertEquals(Set.of(sleeping, active), Set.copyOf(tracker.tickets(List.of(sleeping, active))));
        assertEquals(List.of(sleeping), tracker.tickets(List.of()));
        tracker.remove(sleeping);
        assertTrue(tracker.tickets(List.of()).isEmpty());
    }

    @Test void migrationAndDisposalPreserveImmutableSnapshots() {
        ResidentBodyTracker<Object> tracker = new ResidentBodyTracker<>();
        Object body = new Object();
        tracker.add(body);
        List<Object> before = tracker.tickets(List.of());
        tracker.remove(body); tracker.add(body); // Transfer between regions.
        assertEquals(before, tracker.tickets(List.of()));
        tracker.clear();
        assertTrue(tracker.tickets(List.of()).isEmpty());
        assertEquals(List.of(body), before);
    }
}
