package com.nstut.worldengine.physics;

import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.objects.ObjectArrayList;
import it.unimi.dsi.fastutil.objects.Reference2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceOpenHashSet;
import net.minecraft.core.SectionPos;

/** Incremental section index used because Sable 2.0.4 compiles its optional ticket-query index out. */
public final class WorldEngineBodyIndex {
    private static final long MAX_ENUMERATED_QUERY_SECTIONS = 4096L;

    private final Long2ObjectOpenHashMap<ReferenceOpenHashSet<ServerSubLevel>> sections =
            new Long2ObjectOpenHashMap<>();
    private final Reference2ObjectOpenHashMap<ServerSubLevel, LongOpenHashSet> bodySections =
            new Reference2ObjectOpenHashMap<>();

    public void update(ServerSubLevel body) {
        LongOpenHashSet previous = this.bodySections.get(body);
        LongOpenHashSet current = this.sectionsFor(body.boundingBox());
        if (current.equals(previous)) return;

        if (previous != null) {
            for (long section : previous) this.removeFromSection(section, body);
        }
        for (long section : current) {
            this.sections.computeIfAbsent(section, ignored -> new ReferenceOpenHashSet<>()).add(body);
        }
        this.bodySections.put(body, current);
    }

    public void remove(ServerSubLevel body) {
        LongOpenHashSet previous = this.bodySections.remove(body);
        if (previous == null) return;
        for (long section : previous) this.removeFromSection(section, body);
    }

    public Iterable<SubLevel> query(BoundingBox3dc bounds) {
        ReferenceOpenHashSet<ServerSubLevel> candidates = new ReferenceOpenHashSet<>();
        BoundingBox3i chunks = bounds.chunkBoundsFrom();
        if (canEnumerateQuerySections(chunks)) {
            for (long section : this.sectionsFor(chunks)) {
                ReferenceOpenHashSet<ServerSubLevel> residents = this.sections.get(section);
                if (residents != null) candidates.addAll(residents);
            }
        } else {
            candidates.addAll(this.bodySections.keySet());
        }

        ObjectArrayList<SubLevel> result = new ObjectArrayList<>(candidates.size());
        for (ServerSubLevel body : candidates) {
            if (!body.isRemoved() && body.boundingBox().intersects(bounds)) result.add(body);
        }
        return result;
    }

    private LongOpenHashSet sectionsFor(BoundingBox3dc bounds) {
        return this.sectionsFor(bounds.chunkBoundsFrom());
    }

    private LongOpenHashSet sectionsFor(BoundingBox3i chunks) {
        LongOpenHashSet result = new LongOpenHashSet();
        for (int x = chunks.minX(); x <= chunks.maxX(); x++) {
            for (int z = chunks.minZ(); z <= chunks.maxZ(); z++) {
                for (int y = chunks.minY(); y <= chunks.maxY(); y++) {
                    result.add(SectionPos.asLong(x, y, z));
                }
            }
        }
        return result;
    }

    private static boolean canEnumerateQuerySections(BoundingBox3i chunks) {
        long xSections = (long) chunks.maxX() - chunks.minX() + 1L;
        long ySections = (long) chunks.maxY() - chunks.minY() + 1L;
        long zSections = (long) chunks.maxZ() - chunks.minZ() + 1L;
        if (xSections <= 0L || ySections <= 0L || zSections <= 0L) return false;
        if (xSections > MAX_ENUMERATED_QUERY_SECTIONS / ySections) return false;
        return xSections * ySections <= MAX_ENUMERATED_QUERY_SECTIONS / zSections;
    }

    private void removeFromSection(long section, ServerSubLevel body) {
        ReferenceOpenHashSet<ServerSubLevel> residents = this.sections.get(section);
        if (residents == null) return;
        residents.remove(body);
        if (residents.isEmpty()) this.sections.remove(section);
    }
}
