package com.nstut.worldengine.physics;

/** Overflow-safe cost policy shared by production and deterministic tests. */
final class BodyIndexPolicy {
    private static final long MAX_ENUMERATED_QUERY_SECTIONS = 4096L;
    private static final long SECTION_TO_BODY_COST_RATIO = 8L;

    private BodyIndexPolicy() {}

    static boolean shouldEnumerateQuerySections(long sections, int bodyCount) {
        if (sections == Long.MAX_VALUE || sections > MAX_ENUMERATED_QUERY_SECTIONS) return false;
        return sections <= Math.max(1L, (long) bodyCount * SECTION_TO_BODY_COST_RATIO);
    }

    static long sectionVolume(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
        long xSections = (long) maxX - minX + 1L;
        long ySections = (long) maxY - minY + 1L;
        long zSections = (long) maxZ - minZ + 1L;
        if (xSections <= 0L || ySections <= 0L || zSections <= 0L) return Long.MAX_VALUE;
        if (xSections > Long.MAX_VALUE / ySections) return Long.MAX_VALUE;
        long xySections = xSections * ySections;
        if (xySections > Long.MAX_VALUE / zSections) return Long.MAX_VALUE;
        return xySections * zSections;
    }
}
