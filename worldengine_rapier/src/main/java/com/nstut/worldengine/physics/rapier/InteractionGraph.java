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
        final int id;
        int slot;
        double minX, minY, minZ, maxX, maxY, maxZ;
        boolean coversPlane = true;
        StoredBounds(int id, int slot) { this.id = id; this.slot = slot; }
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
    private final java.util.ArrayList<StoredBounds> orderedBounds = new java.util.ArrayList<>();
    private final Int2ObjectMap<IntSet> edges = new Int2ObjectOpenHashMap<>();
    private final Int2ObjectMap<int[]> components = new Int2ObjectOpenHashMap<>();
    private final IntArrayList componentPending = new IntArrayList();
    private final IntSet componentVisited = new IntOpenHashSet();
    private final java.util.function.IntConsumer enqueueComponent = neighbor -> {
        if (componentVisited.add(neighbor)) componentPending.add(neighbor);
    };
    private final IntSet oversized = new IntOpenHashSet();
    private final IntSet candidates = new IntOpenHashSet();
    // Updates are sequential. Snapshot primitive IDs before mutating sets;
    // IntOpenHashSet.forEach visits its table without allocating an iterator.
    private final IntArrayList visits = new IntArrayList();
    private final java.util.function.IntConsumer collectVisit = visits::add;
    private final java.util.function.IntConsumer collectCandidate = candidates::add;
    private double sharedPlaneY;
    private int planeMisses;

    void clear() {
        cells.clear(); ranges.clear(); bounds.clear(); edges.clear(); oversized.clear(); candidates.clear();
        orderedBounds.clear();
        components.clear(); componentPending.clear(); componentVisited.clear();
        planeMisses = 0;
    }

    IntSet neighbors(int id) {
        IntSet result = edges.get(id);
        return result == null ? IntSets.EMPTY_SET : result;
    }

    /** Read-only member IDs, shared until a node or exact edge changes. */
    int[] component(int seed) {
        int[] cached = components.get(seed);
        if (cached != null) return cached;
        if (!bounds.containsKey(seed)) return it.unimi.dsi.fastutil.ints.IntArrays.EMPTY_ARRAY;
        componentPending.clear();
        componentVisited.clear();
        componentPending.add(seed);
        componentVisited.add(seed);
        for (int head = 0; head < componentPending.size(); head++) {
            neighbors(componentPending.getInt(head)).forEach(enqueueComponent);
        }
        int[] members = componentPending.toIntArray();
        for (int id : members) components.put(id, members);
        return members;
    }

    private void invalidateComponents() {
        if (!components.isEmpty()) components.clear();
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
        StoredBounds removed = bounds.remove(id);
        if (removed != null) {
            invalidateComponents();
            StoredBounds last = orderedBounds.removeLast();
            if (last != removed) { orderedBounds.set(removed.slot, last); last.slot = removed.slot; }
        }
        if (removed != null && !removed.coversPlane) planeMisses--;
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
        boolean horizontalUnchanged = fresh != null && fresh.minX == minX && fresh.minZ == minZ
                && fresh.maxX == maxX && fresh.maxZ == maxZ;
        boolean sharedPlaneBefore = planeMisses == 0;
        boolean unchanged = fresh != null && fresh.minX == minX && fresh.minY == minY && fresh.minZ == minZ
                && fresh.maxX == maxX && fresh.maxY == maxY && fresh.maxZ == maxZ;
        if (fresh == null) {
            if (bounds.isEmpty()) sharedPlaneY = minY * 0.5 + maxY * 0.5;
            fresh = new StoredBounds(id, orderedBounds.size());
            orderedBounds.add(fresh);
            bounds.put(id, fresh);
            invalidateComponents();
        }
        boolean coversPlane = minY <= sharedPlaneY && maxY >= sharedPlaneY;
        if (fresh.coversPlane != coversPlane) planeMisses += coversPlane ? -1 : 1;
        fresh.coversPlane = coversPlane;
        fresh.set(minX, minY, minZ, maxX, maxY, maxZ);
        IntSet neighbors = edges.get(id);
        if (neighbors == null) { neighbors = new IntOpenHashSet(); edges.put(id, neighbors); }
        affected.add(id);
        // Other bodies update reciprocal edges when their bounds change. An expired
        // interaction hold still seeds component/migration checks, but identical
        // bounds need no fresh cell or candidate scan.
        if (unchanged) return;
        Range old = ranges.get(id);
        Range range = Range.of(minX, minY, minZ, maxX, maxY, maxZ, old);
        boolean rangeChanged = !java.util.Objects.equals(range, old);
        if (rangeChanged) {
            if (old != null) membership(old, id, false);
            if (range != null) membership(range, id, true);
        }
        if (range == null) { ranges.remove(id); oversized.add(id); }
        else { ranges.put(id, range); oversized.remove(id); }

        // If every stored box covered the same Y plane before and after this
        // update, every pair intersects on Y. Unchanged X/Z bounds then prove
        // that all exact edges survive vertical movement. Keep cell memberships
        // fresh above so a later box outside the plane can still discover them.
        if (horizontalUnchanged && sharedPlaneBefore && coversPlane) return;

        boolean scan = range == null;
        IntSet queryCandidates;
        if (!rangeChanged && range != null && range.cells.length == 1 && oversized.isEmpty()) {
            // This bucket already is the exact candidate union. Edge updates do
            // not mutate cell membership, so snapshot it without a second hash set.
            queryCandidates = cells.get(range.cells[0]);
            scan = queryCandidates.size() >= orderedBounds.size();
        } else {
            candidates.clear();
            oversized.forEach(collectCandidate);
            long candidateVisits = oversized.size();
            if (range != null) for (Cell key : range.cells) {
                IntSet members = cells.get(key);
                if (members == null) continue;
                candidateVisits += members.size();
                // Once bucket visits cost a whole-body scan, bypass deduplication
                // and ID-to-bounds lookups. Every live box is tested exactly once.
                if (candidateVisits >= orderedBounds.size()) { scan = true; break; }
                members.forEach(collectCandidate);
            }
            // A changed cell range can leave old edges outside the new buckets.
            // Unchanged ranges already contain every possible surviving neighbor.
            if (!scan && rangeChanged) neighbors.forEach(collectCandidate);
            queryCandidates = candidates;
        }

        if (scan) {
            for (int i = 0; i < orderedBounds.size(); i++) {
                StoredBounds candidate = orderedBounds.get(i);
                if (candidate.id != id) reconcileEdge(id, fresh, neighbors, candidate, affected);
            }
            return;
        }

        // Test each candidate once for both insertion and deletion of exact edges.
        visits.clear();
        queryCandidates.forEach(collectVisit);
        for (int i = 0; i < visits.size(); i++) {
            int neighbor = visits.getInt(i);
            if (neighbor == id) continue;
            StoredBounds b = bounds.get(neighbor);
            if (b != null) reconcileEdge(id, fresh, neighbors, b, affected);
        }
    }

    private void reconcileEdge(int id, StoredBounds fresh, IntSet neighbors, StoredBounds candidate, IntSet affected) {
        int neighbor = candidate.id;
        if (fresh.intersects(candidate)) {
            if (!neighbors.add(neighbor)) return;
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal == null) { reciprocal = new IntOpenHashSet(); edges.put(neighbor, reciprocal); }
            reciprocal.add(id);
        } else {
            if (!neighbors.remove(neighbor)) return;
            IntSet reciprocal = edges.get(neighbor);
            if (reciprocal != null) reciprocal.remove(id);
        }
        invalidateComponents();
        affected.add(neighbor);
    }
}
