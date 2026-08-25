package com.nstut.worldengine.physics;

import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.objects.Reference2IntOpenHashMap;
import it.unimi.dsi.fastutil.objects.Reference2LongOpenHashMap;
import it.unimi.dsi.fastutil.objects.Reference2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceOpenHashSet;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Reference-keyed spatial index over packed chunk-section coordinates.
 *
 * Queries stream matching bodies directly out of the resident sets: no
 * temporary section sets, candidate sets, or per-query iterators are
 * allocated. Deduplication across overlapping sections uses reusable
 * generation stamps instead of a per-query hash set, so a hot query path
 * (for example every mob movement) performs zero intermediate allocations.
 *
 * Section packing must match {@code SectionPos.asLong(x, y, z)}:
 * x 26 bits << 38, z 26 bits << 12, y 12 bits.
 */
public final class SectionSpatialIndex<T> {
    private static final int STAMP_HYGIENE_FACTOR = 4;
    private static final int STAMP_HYGIENE_SLACK = 64;

    private final Long2ObjectOpenHashMap<ReferenceOpenHashSet<T>> sections =
            new Long2ObjectOpenHashMap<>();
    private final Reference2ObjectOpenHashMap<T, LongOpenHashSet> bodySections =
            new Reference2ObjectOpenHashMap<>();
    private final Reference2LongOpenHashMap<T> resultStamps = new Reference2LongOpenHashMap<>();
    private long stampGeneration = 0L;

    public SectionSpatialIndex() {
        this.resultStamps.defaultReturnValue(0L);
    }

    public static long packSection(int x, int y, int z) {
        return ((long) (x & 0x3FFFFFF) << 38) | ((long) (z & 0x3FFFFFF) << 12) | (y & 0xFFF);
    }

    public boolean isEmpty() {
        return this.bodySections.isEmpty();
    }

    public int size() {
        return this.bodySections.size();
    }

    public LongOpenHashSet sectionsOf(T body) {
        return this.bodySections.get(body);
    }

    /**
     * Indexes {@code body} over {@code sectionsOccupied}, taking ownership of the set.
     * The caller must not reuse the instance afterwards.
     */
    public void insert(T body, LongOpenHashSet sectionsOccupied) {
        LongOpenHashSet previous = this.bodySections.put(body, sectionsOccupied);
        if (previous != null) {
            for (long section : previous) this.removeFromSection(section, body);
        }
        for (long section : sectionsOccupied) {
            this.sections.computeIfAbsent(section, ignored -> new ReferenceOpenHashSet<>()).add(body);
        }
    }

    public void remove(T body) {
        LongOpenHashSet previous = this.bodySections.remove(body);
        this.resultStamps.remove(body);
        if (previous == null) return;
        for (long section : previous) this.removeFromSection(section, body);
    }

    /**
     * Collects unique bodies passing {@code filter} whose sections lie inside the
     * inclusive section-coordinate ranges. Returns a shared immutable empty list
     * when nothing matches.
     */
    public List<T> querySections(int minX, int minY, int minZ, int maxX, int maxY, int maxZ,
            Predicate<T> filter) {
        List<T> result = null;
        final long generation = ++this.stampGeneration;
        for (int x = minX; x <= maxX; x++) {
            for (int z = minZ; z <= maxZ; z++) {
                for (int y = minY; y <= maxY; y++) {
                    ReferenceOpenHashSet<T> residents = this.sections.get(packSection(x, y, z));
                    if (residents == null) continue;
                    for (T body : residents) {
                        if (this.resultStamps.getLong(body) == generation) continue;
                        this.resultStamps.put(body, generation);
                        if (filter.test(body)) {
                            if (result == null) result = new ArrayList<>(residents.size());
                            result.add(body);
                        }
                    }
                }
            }
        }
        this.hygiene();
        return result == null ? List.of() : result;
    }

    /**
     * Fallback for bounds too large to enumerate: scans every indexed body once.
     * No deduplication is needed because each body appears exactly once.
     */
    public List<T> queryAll(Predicate<T> filter) {
        List<T> result = null;
        for (T body : this.bodySections.keySet()) {
            if (filter.test(body)) {
                if (result == null) result = new ArrayList<>(this.bodySections.size());
                result.add(body);
            }
        }
        return result == null ? List.of() : result;
    }

    private void hygiene() {
        if (this.stampGeneration == Long.MAX_VALUE || this.resultStamps.size()
                > STAMP_HYGIENE_FACTOR * this.bodySections.size() + STAMP_HYGIENE_SLACK) {
            this.stampGeneration = 0L;
            this.resultStamps.clear();
        }
    }

    private void removeFromSection(long section, T body) {
        ReferenceOpenHashSet<T> residents = this.sections.get(section);
        if (residents == null) return;
        residents.remove(body);
        if (residents.isEmpty()) this.sections.remove(section);
    }
}
