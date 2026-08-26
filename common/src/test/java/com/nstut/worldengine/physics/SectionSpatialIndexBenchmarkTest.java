package com.nstut.worldengine.physics;

import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.objects.ObjectArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceOpenHashSet;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Correctness parity and performance comparison for {@link SectionSpatialIndex}
 * against the previous allocation-heavy query implementation (inlined below as
 * {@code LegacyIndex}), which mirrors WorldEngineBodyIndex before the rewrite.
 *
 * Both sides receive the same injected packer, mirroring how production wires
 * {@code SectionPos::asLong} into insert and query. The packer below replicates
 * the real 1.21.1 SectionPos layout (x 22 bits << 42, z 22 bits << 20,
 * y 20 bits << 0) so key behaviour matches production; parity would hold for
 * any injectable layout because both paths share it.
 */
class SectionSpatialIndexBenchmarkTest {

    private static final SectionSpatialIndex.Packer PACKER =
            (x, y, z) -> ((long) (x & 0x3FFFFF) << 42) | ((long) (z & 0x3FFFFF) << 20)
                    | ((long) (y & 0xFFFFF));

    static final class FakeBody {
        final int minX, minY, minZ, maxX, maxY, maxZ;
        boolean removed;

        FakeBody(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
            this.minX = minX; this.minY = minY; this.minZ = minZ;
            this.maxX = maxX; this.maxY = maxY; this.maxZ = maxZ;
        }

        boolean intersects(FakeBody other) {
            return this.minX <= other.maxX && this.maxX >= other.minX
                    && this.minY <= other.maxY && this.maxY >= other.minY
                    && this.minZ <= other.maxZ && this.maxZ >= other.minZ;
        }
    }

    /** Faithful copy of the pre-rewrite WorldEngineBodyIndex behaviour. */
    static final class LegacyIndex {
        private final Long2ObjectOpenHashMap<ReferenceOpenHashSet<FakeBody>> sections =
                new Long2ObjectOpenHashMap<>();
        final ReferenceOpenHashSet<FakeBody> allBodies = new ReferenceOpenHashSet<>();

        void insert(FakeBody body) {
            this.allBodies.add(body);
            this.forEachSection(body, key ->
                    this.sections.computeIfAbsent(key, k -> new ReferenceOpenHashSet<>()).add(body));
        }

        void remove(FakeBody body) {
            this.allBodies.remove(body);
            this.forEachSection(body, key -> {
                var set = this.sections.get(key);
                if (set != null) {
                    set.remove(body);
                    if (set.isEmpty()) this.sections.remove(key);
                }
            });
        }

        List<FakeBody> query(FakeBody bounds) {
            ReferenceOpenHashSet<FakeBody> candidates = new ReferenceOpenHashSet<>();
            LongOpenHashSet queried = new LongOpenHashSet();
            this.forEachSection(bounds, queried::add);
            for (long section : queried) {
                var residents = this.sections.get(section);
                if (residents != null) candidates.addAll(residents);
            }
            List<FakeBody> result = new ObjectArrayList<>(candidates.size());
            for (FakeBody body : candidates) {
                if (!body.removed && body.intersects(bounds)) result.add(body);
            }
            return result;
        }

        private void forEachSection(FakeBody body, java.util.function.LongConsumer sink) {
            for (int x = body.minX; x <= body.maxX; x++)
                for (int z = body.minZ; z <= body.maxZ; z++)
                    for (int y = body.minY; y <= body.maxY; y++)
                        sink.accept(PACKER.pack(x, y, z));
        }
    }

    private static Predicate<FakeBody> liveFilter(FakeBody bounds) {
        return body -> !body.removed && body.intersects(bounds);
    }

    private static void populate(SectionSpatialIndex<FakeBody> spatial, FakeBody body) {
        LongOpenHashSet secs = new LongOpenHashSet();
        for (int x = body.minX; x <= body.maxX; x++)
            for (int z = body.minZ; z <= body.maxZ; z++)
                for (int y = body.minY; y <= body.maxY; y++)
                    secs.add(PACKER.pack(x, y, z));
        spatial.insert(body, secs);
    }

    @Test
    void emptyIndexReturnsSharedEmptyList() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        assertTrue(index.isEmpty());
        assertSame(List.of(), index.querySections(-4, -4, -4, 4, 4, 4, b -> true));
        assertSame(List.of(), index.queryAll(b -> true));
    }

    @Test
    void singleSectionSingleBodyFastCase() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        FakeBody body = new FakeBody(3, -2, 5, 3, -2, 5);
        populate(index, body);

        List<FakeBody> hit = index.querySections(0, -16, 0, 8, 8, 8, liveFilter(
                new FakeBody(0, -16, 0, 8, 8, 8)));
        assertEquals(1, hit.size());
        assertSame(body, hit.get(0));

        FakeBody miss = new FakeBody(100, 100, 100, 101, 101, 101);
        assertSame(List.of(), index.querySections(miss.minX, miss.minY, miss.minZ,
                miss.maxX, miss.maxY, miss.maxZ, liveFilter(miss)));
    }

    /**
     * Mirrors production: bodies are inserted through one path and queried
     * through another, sharing only the injected packer.
     */
    @Test
    void insertAndQueryShareProductionPackerIdentity() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        LegacyIndex legacy = new LegacyIndex();
        Random random = new Random(99);

        for (int i = 0; i < 50; i++) {
            int x = random.nextInt(400_000) - 200_000;
            int y = random.nextInt(1_000_000) >> 10;
            int z = random.nextInt(400_000) - 200_000;
            FakeBody body = new FakeBody(x, y, z, x, y + 1, z);
            populate(index, body);
            legacy.insert(body);

            List<FakeBody> actual = index.querySections(x - 2, y - 2, z - 2,
                    x + 2, y + 2, z + 2, liveFilter(body));
            assertEquals(1, actual.size());
            assertEquals(legacy.query(body).size(), actual.size());
        }
    }

    @RepeatedTest(8)
    void parityWithLegacyImplementation() {
        Random random = new Random(42);
        SectionSpatialIndex<FakeBody> spatial = new SectionSpatialIndex<>(PACKER);
        LegacyIndex legacy = new LegacyIndex();
        Set<FakeBody> tracked = new HashSet<>();

        for (int round = 0; round < 60; round++) {
            if (!tracked.isEmpty() && random.nextInt(4) == 0) {
                FakeBody victim = tracked.iterator().next();
                tracked.remove(victim);
                spatial.remove(victim);
                legacy.remove(victim);
            }
            if (random.nextInt(6) == 0 && !tracked.isEmpty()) {
                FakeBody removed = tracked.iterator().next();
                removed.removed = true;
            } else {
                int x = random.nextInt(40) - 20, y = random.nextInt(20) - 10, z = random.nextInt(40) - 20;
                FakeBody body = new FakeBody(x, y, z,
                        x + random.nextInt(3), y + random.nextInt(3), z + random.nextInt(3));
                if (tracked.add(body)) {
                    populate(spatial, body);
                    legacy.insert(body);
                }
            }

            int qx = random.nextInt(50) - 25, qy = random.nextInt(30) - 15, qz = random.nextInt(50) - 25;
            FakeBody query = new FakeBody(qx, qy, qz,
                    qx + random.nextInt(6), qy + random.nextInt(6), qz + random.nextInt(6));

            List<FakeBody> expected = new ArrayList<>(legacy.query(query));
            List<FakeBody> actual = spatial.querySections(query.minX, query.minY, query.minZ,
                    query.maxX, query.maxY, query.maxZ, liveFilter(query));

            assertEquals(new HashSet<>(expected), new HashSet<>(actual),
                    "query mismatch in round " + round);
            assertEquals(expected.size(), actual.size(), "duplicate bodies leaked");
        }
    }

    @Test
    void worldScaleFallbackMatchesFullScan() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        FakeBody body = new FakeBody(-40, -64, -40, 40, 320, 40);
        populate(index, body);
        FakeBody other = new FakeBody(500_000, 0, 500_000, 500_001, 0, 500_001);
        populate(index, other);

        FakeBody worldBounds = new FakeBody(Integer.MIN_VALUE / 2, Integer.MIN_VALUE / 2,
                Integer.MIN_VALUE / 2, Integer.MAX_VALUE / 2, Integer.MAX_VALUE / 2, Integer.MAX_VALUE / 2);
        List<FakeBody> hits = index.queryAll(liveFilter(worldBounds));
        assertEquals(2, hits.size());
        assertTrue(hits.contains(body));
        assertTrue(hits.contains(other));

        FakeBody farAway = new FakeBody(1_000_000_000, 0, 1_000_000_000,
                1_000_000_001, 1, 1_000_000_001);
        assertSame(List.of(), index.queryAll(liveFilter(farAway)));
    }

    @Test
    void stampHygieneClearsAfterRemovals() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        List<FakeBody> bodies = new ArrayList<>();
        for (int i = 0; i < 128; i++) {
            FakeBody body = new FakeBody(i * 2, 0, i * 2, i * 2 + 1, 0, i * 2 + 1);
            bodies.add(body);
            populate(index, body);
        }
        for (int q = 0; q < 200; q++) {
            index.querySections(0, -1, 0, 300, 1, 300, b -> false);
        }
        for (FakeBody body : bodies) index.remove(body);
        assertTrue(index.isEmpty());
        for (FakeBody body : bodies) populate(index, body);
        assertEquals(bodies.size(), index.size());
    }

    /**
     * Models the production shape: a handful of multi-section sublevels is
     * indexed, then thousands of mob-sized queries hit the index each tick.
     */
    @Test
    void benchmarkSublevelIndexManyMobQueries() {
        Random random = new Random(7);
        SectionSpatialIndex<FakeBody> spatial = new SectionSpatialIndex<>(PACKER);
        LegacyIndex legacy = new LegacyIndex();

        final int subLevelCount = 24;
        for (int i = 0; i < subLevelCount; i++) {
            int x = random.nextInt(160) - 80, y = random.nextInt(8), z = random.nextInt(160) - 80;
            int span = 1 + random.nextInt(2);
            FakeBody subLevel = new FakeBody(x, y, z, x + span, y + span, z + span);
            populate(spatial, subLevel);
            legacy.insert(subLevel);
        }

        final int mobCount = 4_000;
        final int ticks = 5;
        FakeBody[] mobs = new FakeBody[mobCount];
        for (int i = 0; i < mobCount; i++) {
            int x = random.nextInt(180) - 90, z = random.nextInt(180) - 90;
            mobs[i] = new FakeBody(x, 0, z, x + 1, 1, z + 1);
        }

        for (int warmup = 0; warmup < 3; warmup++) runAll(spatial, mobs, ticks);
        runAll(legacy, mobs, ticks);
        runAll(legacy, mobs, ticks);

        long legacyStart = System.nanoTime();
        runAll(legacy, mobs, ticks);
        long legacyNanos = System.nanoTime() - legacyStart;

        long newStart = System.nanoTime();
        runAll(spatial, mobs, ticks);
        long newNanos = System.nanoTime() - newStart;

        long totalQueries = (long) mobCount * ticks;
        double legacyMs = legacyNanos / 1_000_000.0;
        double newMs = newNanos / 1_000_000.0;
        System.out.printf("=== WorldEngine body-index benchmark (%d sublevels, %d mobs x %d ticks) ===%n",
                subLevelCount, mobCount, ticks);
        System.out.printf("legacy (allocating): %.2f ms  (%.2f us/query)%n", legacyMs,
                legacyNanos / 1000.0 / totalQueries);
        System.out.printf("streamed (stamped):  %.2f ms  (%.2f us/query)%n", newMs,
                newNanos / 1000.0 / totalQueries);
        System.out.printf("speedup: %.2fx%n", legacyNanos / (double) Math.max(1, newNanos));

        assertTrue(newNanos <= legacyNanos * 3 / 2 + 2_000_000L,
                "streamed query regressed: " + newMs + "ms vs " + legacyMs + "ms baseline");
    }

    private static long sink;

    private static void runAll(SectionSpatialIndex<FakeBody> index, FakeBody[] mobs, int ticks) {
        long acc = 0;
        for (int t = 0; t < ticks; t++) {
            for (FakeBody mob : mobs) {
                acc += index.querySections(mob.minX, mob.minY, mob.minZ,
                        mob.maxX, mob.maxY, mob.maxZ, liveFilter(mob)).size();
            }
        }
        sink += acc;
    }

    private static void runAll(LegacyIndex index, FakeBody[] mobs, int ticks) {
        long acc = 0;
        for (int t = 0; t < ticks; t++) {
            for (FakeBody mob : mobs) {
                acc += index.query(mob).size();
            }
        }
        sink += acc;
    }
}
