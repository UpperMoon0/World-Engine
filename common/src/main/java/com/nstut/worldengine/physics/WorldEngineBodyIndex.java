package com.nstut.worldengine.physics;

import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import net.minecraft.core.SectionPos;

import java.util.List;

/**
 * Incremental section index used because Sable 2.0.4 compiles its optional ticket-query index out.
 *
 * Queries avoid intermediate allocations for the common cases: an empty index returns
 * a shared immutable list, section enumeration streams matches through generation stamps
 * (see {@link SectionSpatialIndex}) using one reusable predicate instance, and the query
 * chunk bounds reuse a scratch box. Only the match list allocates, lazily on first match.
 * Insertion and query keys both come from {@link SectionPos#asLong}, so they cannot drift.
 */
public final class WorldEngineBodyIndex {
    private static final long MAX_ENUMERATED_QUERY_SECTIONS = 4096L;

    private final SectionSpatialIndex<ServerSubLevel> index =
            new SectionSpatialIndex<>(SectionPos::asLong);
    private final BoundingBox3i scratchChunks = new BoundingBox3i();
    private final BodyFilter filter = new BodyFilter();
    private LongOpenHashSet sectionsScratch = new LongOpenHashSet();

    public void update(ServerSubLevel body) {
        LongOpenHashSet previous = this.index.sectionsOf(body);
        this.sectionsScratch.clear();
        collectSections(body.boundingBox().chunkBoundsFrom(this.scratchChunks), this.sectionsScratch);
        if (previous != null && previous.equals(this.sectionsScratch)) return;

        LongOpenHashSet fresh = this.sectionsScratch;
        this.sectionsScratch = new LongOpenHashSet();
        this.index.insert(body, fresh);
    }

    public void remove(ServerSubLevel body) {
        this.index.remove(body);
    }

    public Iterable<SubLevel> query(BoundingBox3dc bounds) {
        if (this.index.isEmpty()) return List.of();

        BoundingBox3i chunks = bounds.chunkBoundsFrom(this.scratchChunks);
        BodyFilter live = this.filter.forBounds(bounds);
        List<ServerSubLevel> matches;
        if (canEnumerateQuerySections(chunks)) {
            matches = this.index.querySections(chunks.minX(), chunks.minY(), chunks.minZ(),
                    chunks.maxX(), chunks.maxY(), chunks.maxZ(), live);
        } else {
            matches = this.index.queryAll(live);
        }
        @SuppressWarnings("unchecked")
        Iterable<SubLevel> result = (Iterable<SubLevel>) (Object) matches;
        return result;
    }

    /**
     * Reusable per-index predicate so hot queries do not allocate a capturing
     * lambda. Not reentrant: bounds must not change during a query, which the
     * collision pipeline never does inside an intersects test.
     */
    private final class BodyFilter implements java.util.function.Predicate<ServerSubLevel> {
        private BoundingBox3dc bounds;

        BodyFilter forBounds(BoundingBox3dc value) {
            this.bounds = value;
            return this;
        }

        @Override
        public boolean test(ServerSubLevel body) {
            return !body.isRemoved() && body.boundingBox().intersects(this.bounds);
        }
    }

    private static void collectSections(BoundingBox3i chunks, LongOpenHashSet dest) {
        for (int x = chunks.minX(); x <= chunks.maxX(); x++) {
            for (int z = chunks.minZ(); z <= chunks.maxZ(); z++) {
                for (int y = chunks.minY(); y <= chunks.maxY(); y++) {
                    dest.add(SectionPos.asLong(x, y, z));
                }
            }
        }
    }

    private static boolean canEnumerateQuerySections(BoundingBox3i chunks) {
        long xSections = (long) chunks.maxX() - chunks.minX() + 1L;
        long ySections = (long) chunks.maxY() - chunks.minY() + 1L;
        long zSections = (long) chunks.maxZ() - chunks.minZ() + 1L;
        if (xSections <= 0L || ySections <= 0L || zSections <= 0L) return false;
        if (xSections > MAX_ENUMERATED_QUERY_SECTIONS / ySections) return false;
        return xSections * ySections <= MAX_ENUMERATED_QUERY_SECTIONS / zSections;
    }
}
