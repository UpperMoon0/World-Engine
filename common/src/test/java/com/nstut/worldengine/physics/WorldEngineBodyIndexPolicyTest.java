package com.nstut.worldengine.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Correctness coverage for the allocation and query cost policies. */
class WorldEngineBodyIndexPolicyTest {
    @Test
    void sectionVolumeIsOverflowSafe() {
        assertEquals(1L, BodyIndexPolicy.sectionVolume(0, 0, 0, 0, 0, 0));
        assertEquals(4096L, BodyIndexPolicy.sectionVolume(0, 0, 0, 15, 15, 15));
        assertEquals(Long.MAX_VALUE, BodyIndexPolicy.sectionVolume(
                Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE,
                Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE));
    }

    @Test
    void queryPolicyAdaptsToBodyCount() {
        assertFalse(BodyIndexPolicy.shouldEnumerateQuerySections(3_000L, 6));
        assertTrue(BodyIndexPolicy.shouldEnumerateQuerySections(4L, 500));
    }

    @Test
    void absoluteQueryLimitStillPreventsWorldScaleEnumeration() {
        long worldScaleSections = BodyIndexPolicy.sectionVolume(
                -30_000_000, -64, -30_000_000, 30_000_000, 320, 30_000_000);
        assertFalse(BodyIndexPolicy.shouldEnumerateQuerySections(
                worldScaleSections, Integer.MAX_VALUE));
    }
}
