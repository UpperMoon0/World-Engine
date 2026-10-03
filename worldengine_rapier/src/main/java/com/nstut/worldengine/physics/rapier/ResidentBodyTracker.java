package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.objects.ReferenceOpenHashSet;
import java.util.List;

/** Cached ticket population; sleeping resident bodies retain terrain support. */
final class ResidentBodyTracker<T> {
    private final ReferenceOpenHashSet<T> residents = new ReferenceOpenHashSet<>();
    private List<T> snapshot = List.of();
    private boolean dirty;

    void add(T body) { if (residents.add(body)) dirty = true; }
    void remove(T body) { if (residents.remove(body)) dirty = true; }
    void clear() { residents.clear(); snapshot = List.of(); dirty = false; }

    List<T> tickets(List<T> active) {
        if (residents.containsAll(active)) {
            if (dirty) { snapshot = List.copyOf(residents); dirty = false; }
            return snapshot;
        }
        ReferenceOpenHashSet<T> union = new ReferenceOpenHashSet<>(residents);
        union.addAll(active);
        return List.copyOf(union);
    }
}
