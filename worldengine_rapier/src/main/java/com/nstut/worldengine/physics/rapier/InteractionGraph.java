package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.*;
import java.util.HashMap;
import java.util.Map;

/** Exact swept-AABB graph. Cell membership and neighbor storage survive pose updates. */
final class InteractionGraph {
    record Bounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        boolean intersects(Bounds b) {
            return minX <= b.maxX && maxX >= b.minX && minY <= b.maxY && maxY >= b.minY
                    && minZ <= b.maxZ && maxZ >= b.minZ;
        }
    }
    private record Cell(int x, int y, int z) {}
    private record Range(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
        static Range of(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, Range previous) {
            int x = coordinate(minX), y = coordinate(minY), z = coordinate(minZ);
            int xx = coordinate(maxX), yy = coordinate(maxY), zz = coordinate(maxZ);
            long sx = (long) xx - x + 1, sy = (long) yy - y + 1, sz = (long) zz - z + 1;
            if (sx <= 0 || sy <= 0 || sz <= 0 || sx > 4096 || sy > 4096 / sx || sz > 4096 / (sx * sy)) return null;
            if (previous != null && previous.minX == x && previous.minY == y && previous.minZ == z
                    && previous.maxX == xx && previous.maxY == yy && previous.maxZ == zz) return previous;
            return new Range(x, y, z, xx, yy, zz);
        }
        private static int coordinate(double value) { return (int) Math.floor(value / 128.0); }
    }
    private static final class StoredBounds {
        double minX, minY, minZ, maxX, maxY, maxZ;
        void set(double x, double y, double z, double xx, double yy, double zz) {
            minX = x; minY = y; minZ = z; maxX = xx; maxY = yy; maxZ = zz;
        }
        boolean intersects(StoredBounds b) {
            return minX <= b.maxX && maxX >= b.minX && minY <= b.maxY && maxY >= b.minY
                    && minZ <= b.maxZ && maxZ >= b.minZ;
        }
    }
    private final Map<Cell, IntSet> cells = new HashMap<>();
    private final Int2ObjectMap<Range> ranges = new Int2ObjectOpenHashMap<>();
    private final Int2ObjectMap<StoredBounds> bounds = new Int2ObjectOpenHashMap<>();
    private final Int2ObjectMap<IntSet> edges = new Int2ObjectOpenHashMap<>();
    private final IntSet oversized = new IntOpenHashSet();
    private final IntSet candidates = new IntOpenHashSet();

    void clear() {
        cells.clear(); ranges.clear(); bounds.clear(); edges.clear(); oversized.clear(); candidates.clear();
    }

    IntSet neighbors(int id) {
        IntSet result = edges.get(id);
        return result == null ? IntSets.EMPTY_SET : result;
    }

    void remove(int id) {
        Range old = ranges.remove(id);
        if (old != null) membership(old, id, false);
        IntSet oldEdges = edges.remove(id);
        if (oldEdges != null) for (int neighbor : oldEdges) {
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal != null) reciprocal.remove(id);
        }
        bounds.remove(id);
        oversized.remove(id);
    }

    private void membership(Range range, int id, boolean add) {
        for (int x = range.minX; x <= range.maxX; x++) for (int y = range.minY; y <= range.maxY; y++)
            for (int z = range.minZ; z <= range.maxZ; z++) {
                Cell key = new Cell(x, y, z);
                IntSet members = cells.get(key);
                if (add) {
                    if (members == null) { members = new IntOpenHashSet(); cells.put(key, members); }
                    members.add(id);
                } else if (members != null) {
                    members.remove(id);
                    if (members.isEmpty()) cells.remove(key);
                }
            }
    }

    void update(int id, Bounds fresh, IntSet affected) {
        update(id, fresh.minX, fresh.minY, fresh.minZ, fresh.maxX, fresh.maxY, fresh.maxZ, affected);
    }

    void update(int id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, IntSet affected) {
        StoredBounds fresh = bounds.get(id);
        if (fresh == null) { fresh = new StoredBounds(); bounds.put(id, fresh); }
        fresh.set(minX, minY, minZ, maxX, maxY, maxZ);
        IntSet neighbors = edges.get(id);
        if (neighbors == null) { neighbors = new IntOpenHashSet(); edges.put(id, neighbors); }
        affected.add(id);
        affected.addAll(neighbors);
        Range old = ranges.get(id);
        Range range = Range.of(minX, minY, minZ, maxX, maxY, maxZ, old);
        if (!java.util.Objects.equals(range, old)) {
            if (old != null) membership(old, id, false);
            if (range != null) membership(range, id, true);
        }
        if (range == null) { ranges.remove(id); oversized.add(id); }
        else { ranges.put(id, range); oversized.remove(id); }

        candidates.clear();
        candidates.addAll(oversized);
        if (range == null) candidates.addAll(bounds.keySet());
        else for (int x = range.minX; x <= range.maxX; x++) for (int y = range.minY; y <= range.maxY; y++)
            for (int z = range.minZ; z <= range.maxZ; z++) {
                IntSet members = cells.get(new Cell(x, y, z));
                if (members != null) candidates.addAll(members);
            }
        candidates.remove(id);

        // Remove stale reciprocal edges in place, then add exact current overlaps.
        for (IntIterator it = neighbors.iterator(); it.hasNext();) {
            int neighbor = it.nextInt();
            StoredBounds b = bounds.get(neighbor);
            if (b != null && fresh.intersects(b)) continue;
            it.remove();
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal != null) reciprocal.remove(id);
        }
        for (int neighbor : candidates) {
            StoredBounds b = bounds.get(neighbor);
            if (b == null || !fresh.intersects(b)) continue;
            neighbors.add(neighbor);
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal == null) { reciprocal = new IntOpenHashSet(); edges.put(neighbor, reciprocal); }
            reciprocal.add(id);
        }
        affected.addAll(neighbors);
    }
}
