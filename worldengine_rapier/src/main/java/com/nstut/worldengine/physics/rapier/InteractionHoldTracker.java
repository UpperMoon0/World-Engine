package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.ints.IntSet;
import java.util.PriorityQueue;

/** One queued deadline per held body; renewal does not enqueue stale copies. */
final class InteractionHoldTracker {
    private static final class Deadline implements Comparable<Deadline> {
        final int id;
        long expiry, scheduled;
        Deadline(int id, long expiry) { this.id = id; this.expiry = expiry; this.scheduled = expiry; }
        public int compareTo(Deadline other) { return Long.compare(scheduled, other.scheduled); }
    }
    private final Int2ObjectOpenHashMap<Deadline> holds = new Int2ObjectOpenHashMap<>();
    private final PriorityQueue<Deadline> queue = new PriorityQueue<>();

    void renew(int id, long expiry) {
        Deadline deadline = holds.get(id);
        if (deadline == null) { deadline = new Deadline(id, expiry); holds.put(id, deadline); queue.add(deadline); }
        else deadline.expiry = expiry; // Heap ordering uses the unchanged scheduled deadline.
    }
    boolean holds(int id, long tick) { Deadline deadline = holds.get(id); return deadline != null && deadline.expiry > tick; }
    void remove(int id) { holds.remove(id); }
    boolean isEmpty() { return queue.isEmpty(); }
    void clear() { holds.clear(); queue.clear(); }

    void drainExpired(long tick, IntSet expired) {
        while (!queue.isEmpty() && queue.peek().scheduled <= tick) {
            Deadline deadline = queue.remove();
            if (holds.get(deadline.id) != deadline) continue;
            if (deadline.expiry > tick) {
                deadline.scheduled = deadline.expiry; // Mutate ordering only while outside the heap.
                queue.add(deadline);
            } else {
                holds.remove(deadline.id);
                expired.add(deadline.id);
            }
        }
    }
}
