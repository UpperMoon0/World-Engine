package com.nstut.worldengine.physics.rapier;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TerrainFootprintTrackerTest {
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
