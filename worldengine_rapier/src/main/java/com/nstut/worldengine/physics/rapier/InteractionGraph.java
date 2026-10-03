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
    private record Range(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, Cell[] cells) {
        static Range of(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, Range previous) {
            int x = coordinate(minX), y = coordinate(minY), z = coordinate(minZ);
            int xx = coordinate(maxX), yy = coordinate(maxY), zz = coordinate(maxZ);
            long sx = (long) xx - x + 1, sy = (long) yy - y + 1, sz = (long) zz - z + 1;
            if (sx <= 0 || sy <= 0 || sz <= 0 || sx > 4096 || sy > 4096 / sx || sz > 4096 / (sx * sy)) return null;
            if (previous != null && previous.minX == x && previous.minY == y && previous.minZ == z
                    && previous.maxX == xx && previous.maxY == yy && previous.maxZ == zz) return previous;
            Cell[] cells = new Cell[(int) (sx * sy * sz)];
            int index = 0;
            for (long cx = x; cx <= xx; cx++) for (long cy = y; cy <= yy; cy++)
                for (long cz = z; cz <= zz; cz++) cells[index++] = new Cell((int) cx, (int) cy, (int) cz);
            return new Range(x, y, z, xx, yy, zz, cells);
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
    // Updates are sequential. Snapshot primitive IDs before mutating sets;
    // IntOpenHashSet.forEach visits its table without allocating an iterator.
    private final IntArrayList visits = new IntArrayList();
    private final java.util.function.IntConsumer collectVisit = visits::add;
    private final java.util.function.IntConsumer collectCandidate = candidates::add;

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
        if (oldEdges != null) for (var neighborIterator = oldEdges.iterator(); neighborIterator.hasNext();) {
            int neighbor = neighborIterator.nextInt();
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal != null) reciprocal.remove(id);
        }
        bounds.remove(id);
        oversized.remove(id);
    }

    private void membership(Range range, int id, boolean add) {
        for (Cell key : range.cells) {
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
        boolean unchanged = fresh != null && fresh.minX == minX && fresh.minY == minY && fresh.minZ == minZ
                && fresh.maxX == maxX && fresh.maxY == maxY && fresh.maxZ == maxZ;
        if (fresh == null) { fresh = new StoredBounds(); bounds.put(id, fresh); }
        fresh.set(minX, minY, minZ, maxX, maxY, maxZ);
        IntSet neighbors = edges.get(id);
        if (neighbors == null) { neighbors = new IntOpenHashSet(); edges.put(id, neighbors); }
        affected.add(id);
        visits.clear();
        neighbors.forEach(collectVisit);
        for (int i = 0; i < visits.size(); i++) affected.add(visits.getInt(i));
        // Other bodies update reciprocal edges when their bounds change. An expired
        // interaction hold still seeds component/migration checks, but identical
        // bounds need no fresh cell or candidate scan.
        if (unchanged) return;
        Range old = ranges.get(id);
        Range range = Range.of(minX, minY, minZ, maxX, maxY, maxZ, old);
        if (!java.util.Objects.equals(range, old)) {
            if (old != null) membership(old, id, false);
            if (range != null) membership(range, id, true);
        }
        if (range == null) { ranges.remove(id); oversized.add(id); }
        else { ranges.put(id, range); oversized.remove(id); }

        candidates.clear();
        oversized.forEach(collectCandidate);
        if (range == null) bounds.keySet().forEach(collectCandidate);
        else for (Cell key : range.cells) {
            IntSet members = cells.get(key);
            if (members != null) members.forEach(collectCandidate);
        }
        candidates.remove(id);

        // Remove stale reciprocal edges in place, then add exact current overlaps.
        for (int i = 0; i < visits.size(); i++) {
            int neighbor = visits.getInt(i);
            StoredBounds b = bounds.get(neighbor);
            if (b != null && fresh.intersects(b)) continue;
            neighbors.remove(neighbor);
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal != null) reciprocal.remove(id);
        }
        visits.clear();
        candidates.forEach(collectVisit);
        for (int i = 0; i < visits.size(); i++) {
            int neighbor = visits.getInt(i);
            // Surviving edges were already tested against these exact fresh
            // bounds above. Their reciprocal edge is already present.
            if (neighbors.contains(neighbor)) continue;
            StoredBounds b = bounds.get(neighbor);
            if (b == null || !fresh.intersects(b)) continue;
            neighbors.add(neighbor);
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal == null) { reciprocal = new IntOpenHashSet(); edges.put(neighbor, reciprocal); }
            reciprocal.add(id);
            affected.add(neighbor);
        }
    }
}
