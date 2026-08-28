package com.nstut.worldengine.physics;

import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Fast, deterministic correctness tests for {@link SectionSpatialIndex}. */
class SectionSpatialIndexTest {
    private static final SectionSpatialIndex.Packer PACKER =
            (x, y, z) -> ((long) (x & 0x3FFFFF) << 42) | ((long) (z & 0x3FFFFF) << 20)
                    | ((long) y & 0xFFFFF);

    private static final class FakeBody {
        final int minX;
        final int minY;
        final int minZ;
        final int maxX;
        final int maxY;
        final int maxZ;
        boolean removed;

        FakeBody(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
            this.minX = minX;
            this.minY = minY;
            this.minZ = minZ;
            this.maxX = maxX;
            this.maxY = maxY;
            this.maxZ = maxZ;
        }

        boolean intersects(FakeBody other) {
            return this.minX <= other.maxX && this.maxX >= other.minX
                    && this.minY <= other.maxY && this.maxY >= other.minY
                    && this.minZ <= other.maxZ && this.maxZ >= other.minZ;
        }
    }

    private static Predicate<FakeBody> liveFilter(FakeBody bounds) {
        return body -> !body.removed && body.intersects(bounds);
    }

    private static LongOpenHashSet sections(FakeBody body) {
        LongOpenHashSet result = new LongOpenHashSet();
        for (int x = body.minX; x <= body.maxX; x++) {
            for (int z = body.minZ; z <= body.maxZ; z++) {
                for (int y = body.minY; y <= body.maxY; y++) {
                    result.add(PACKER.pack(x, y, z));
                }
            }
        }
        return result;
    }

    private static void insert(SectionSpatialIndex<FakeBody> index, FakeBody body) {
        index.insert(body, sections(body));
    }

    @Test
    void emptyIndexReturnsSharedEmptyList() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        assertTrue(index.isEmpty());
        assertSame(List.of(), index.querySections(-1, -1, -1, 1, 1, 1, body -> true));
        assertSame(List.of(), index.queryAll(body -> true));
    }

    @Test
    void indexedQueryDeduplicatesMultiSectionBodies() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        FakeBody body = new FakeBody(-1, -1, -1, 1, 1, 1);
        insert(index, body);

        List<FakeBody> matches = index.querySections(-2, -2, -2, 2, 2, 2, liveFilter(body));
        assertEquals(List.of(body), matches);
    }

    @Test
    void largeBodiesAreExactAndDoNotCreateSectionMemberships() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        FakeBody large = new FakeBody(-1_000_000, -10, -1_000_000,
                1_000_000, 10, 1_000_000);
        index.insertLarge(large);

        assertTrue(index.isLarge(large));
        assertNull(index.sectionsOf(large));
        assertEquals(List.of(large), index.querySections(0, 0, 0, 0, 0, 0,
                liveFilter(new FakeBody(0, 0, 0, 0, 0, 0))));
        assertSame(List.of(), index.querySections(0, 100, 0, 0, 100, 0,
                liveFilter(new FakeBody(0, 100, 0, 0, 100, 0))));
    }

    @Test
    void bodiesCanTransitionBetweenIndexTiers() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        FakeBody body = new FakeBody(0, 0, 0, 0, 0, 0);
        insert(index, body);
        index.insertLarge(body);
        assertTrue(index.isLarge(body));
        assertSame(List.of(), index.querySections(0, 0, 0, 0, 0, 0, candidate -> false));

        index.insert(body, sections(body));
        assertFalse(index.isLarge(body));
        assertEquals(List.of(body), index.queryAll(candidate -> true));

        index.remove(body);
        assertTrue(index.isEmpty());
    }

    @Test
    void randomizedQueriesMatchLinearBodyScan() {
        Random random = new Random(42L);
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        List<FakeBody> bodies = new ArrayList<>();

        for (int i = 0; i < 80; i++) {
            int x = random.nextInt(50) - 25;
            int y = random.nextInt(20) - 10;
            int z = random.nextInt(50) - 25;
            FakeBody body = new FakeBody(x, y, z, x + random.nextInt(3),
                    y + random.nextInt(3), z + random.nextInt(3));
            bodies.add(body);
            if (i % 17 == 0) index.insertLarge(body);
            else insert(index, body);
        }

        for (int i = 0; i < 100; i++) {
            int x = random.nextInt(60) - 30;
            int y = random.nextInt(30) - 15;
            int z = random.nextInt(60) - 30;
            FakeBody query = new FakeBody(x, y, z, x + random.nextInt(5),
                    y + random.nextInt(5), z + random.nextInt(5));
            Set<FakeBody> expected = new HashSet<>();
            for (FakeBody body : bodies) if (body.intersects(query)) expected.add(body);

            Set<FakeBody> actual = new HashSet<>(index.querySections(query.minX, query.minY,
                    query.minZ, query.maxX, query.maxY, query.maxZ, liveFilter(query)));
            assertEquals(expected, actual);
        }
    }

    @Test
    void removalsDoNotLeakQueryStamps() {
        SectionSpatialIndex<FakeBody> index = new SectionSpatialIndex<>(PACKER);
        List<FakeBody> bodies = new ArrayList<>();
        for (int i = 0; i < 64; i++) {
            FakeBody body = new FakeBody(i, 0, i, i, 0, i);
            bodies.add(body);
            insert(index, body);
        }
        for (int query = 0; query < 20; query++) {
            index.querySections(0, 0, 0, 64, 0, 64, candidate -> false);
        }
        for (FakeBody body : bodies) index.remove(body);
        assertTrue(index.isEmpty());
    }
}
