package com.nstut.worldengine.physics.rapier;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;

class TerrainFootprintTrackerTest {
    @Test
    void sameSectionMotionReusesEnvelopeButBoundaryAndForcedRefreshDoNot() {
        TerrainFootprintTracker tracker = new TerrainFootprintTracker();
        var first = TerrainFootprintTracker.Envelope.fromWorldBounds(-15, 64, 1, -1, 70, 15);
        assertTrue(tracker.needsRefresh(7, first));
        var sameSections = TerrainFootprintTracker.Envelope.fromWorldBounds(-14, 65, 2, -2, 71, 14,
                tracker.previousEnvelope(7));
        assertSame(first, sameSections);
        assertFalse(tracker.needsRefresh(7, sameSections));
        var crossing = TerrainFootprintTracker.Envelope.fromWorldBounds(-17, 65, 2, -2, 71, 14, first);
        assertNotSame(first, crossing);
        assertTrue(tracker.needsRefresh(7, crossing));
        tracker.forceDirty(7);
        assertNull(tracker.previousEnvelope(7));
        assertTrue(tracker.needsRefresh(7, crossing));
    }
    @Test
    void unchangedEnvelopeDoesNotDirtyBodyAgain() {
        TerrainFootprintTracker tracker = new TerrainFootprintTracker();
        TerrainFootprintTracker.Envelope envelope =
                TerrainFootprintTracker.Envelope.fromWorldBounds(1.25, 64.0, 1.25, 15.75, 70.0, 15.75);

        tracker.forceDirty(7);
        assertArrayEquals(new int[]{7}, tracker.drainDirtyBodies());
        assertTrue(tracker.needsRefresh(7, envelope));

        tracker.markDirty(7);
        assertArrayEquals(new int[]{7}, tracker.drainDirtyBodies());
        assertFalse(tracker.needsRefresh(7, envelope));
    }

    @Test
    void crossingSectionBoundaryDirtiesBody() {
        TerrainFootprintTracker tracker = new TerrainFootprintTracker();
        assertTrue(tracker.needsRefresh(7,
                TerrainFootprintTracker.Envelope.fromWorldBounds(1.0, 64.0, 1.0, 15.0, 70.0, 15.0)));

        assertTrue(tracker.needsRefresh(7,
                TerrainFootprintTracker.Envelope.fromWorldBounds(2.0, 64.0, 1.0, 16.0, 70.0, 15.0)));
    }

    @Test
    void invalidOrOversizedBoundsProduceEmptyEnvelope() {
        assertTrue(TerrainFootprintTracker.Envelope.fromWorldBounds(
                Double.NaN, 0.0, 0.0, 1.0, 1.0, 1.0).isEmpty());
        assertTrue(TerrainFootprintTracker.Envelope.fromWorldBounds(
                0.0, 0.0, 0.0, 65536.0, 65536.0, 65536.0).isEmpty());
    }

    @Test
    void resetForcesAllActiveBodiesToRefresh() {
        TerrainFootprintTracker tracker = new TerrainFootprintTracker();
        tracker.reset(new int[]{3, 5});

        int[] dirty = tracker.drainDirtyBodies();
        java.util.Arrays.sort(dirty);
        assertArrayEquals(new int[]{3, 5}, dirty);
    }
}
