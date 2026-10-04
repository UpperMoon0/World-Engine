package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.ints.IntSet;
import java.util.PriorityQueue;

/** Reuse renewed deadlines during reconciliation; only live holds are queued. */
final class InteractionHoldTracker {
    private static final class Deadline implements Comparable<Deadline> {
        final int id;
        long expiry, scheduled;
        boolean queued = true;
        Deadline(int id, long expiry) { this.id = id; this.expiry = expiry; this.scheduled = expiry; }
        public int compareTo(Deadline other) { return Long.compare(scheduled, other.scheduled); }
    }
    private final Int2ObjectOpenHashMap<Deadline> holds = new Int2ObjectOpenHashMap<>();
    private final PriorityQueue<Deadline> queue = new PriorityQueue<>();
    private final java.util.ArrayList<Deadline> expiredDeadlines = new java.util.ArrayList<>();

    void renew(int id, long expiry) {
        Deadline deadline = holds.get(id);
        if (deadline == null) { deadline = new Deadline(id, expiry); holds.put(id, deadline); queue.add(deadline); }
        else {
            deadline.expiry = expiry;
            if (!deadline.queued) {
                deadline.scheduled = expiry;
                deadline.queued = true;
                queue.add(deadline);
            } else if (expiry < deadline.scheduled) {
                // An earlier replacement must also be observed on time. Remove
                // before changing the heap key; normal extensions avoid this scan.
                queue.remove(deadline);
                deadline.scheduled = expiry;
                queue.add(deadline);
            }
            // A queued deadline keeps its scheduled value until it leaves the heap.
        }
    }
    boolean holds(int id, long tick) { Deadline deadline = holds.get(id); return deadline != null && deadline.expiry > tick; }
    void remove(int id) { holds.remove(id); }
    boolean isEmpty() { return queue.isEmpty(); }
    void clear() { holds.clear(); queue.clear(); expiredDeadlines.clear(); }

    void finishExpiryReconciliation() {
        for (Deadline deadline : expiredDeadlines) {
            if (!deadline.queued && holds.get(deadline.id) == deadline) holds.remove(deadline.id);
        }
        expiredDeadlines.clear();
    }

    void drainExpired(long tick, IntSet expired) {
        while (!queue.isEmpty() && queue.peek().scheduled <= tick) {
            Deadline deadline = queue.remove();
            if (holds.get(deadline.id) != deadline) continue;
            if (deadline.expiry > tick) {
                deadline.scheduled = deadline.expiry; // Mutate ordering only while outside the heap.
                queue.add(deadline);
            } else {
                // Reconciliation can renew an unchanged interaction immediately.
                // Retain it for this reconciliation without keeping it live or queued.
                deadline.queued = false;
                expiredDeadlines.add(deadline);
                expired.add(deadline.id);
            }
        }
    }
}
