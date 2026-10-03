package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.longs.LongSet;
import java.util.function.LongConsumer;

/** Updates owned storage without retaining the caller's reusable desired set. */
final class TerrainFootprintDiff {
    static void update(LongSet stored, LongSet desired, LongConsumer removed, LongConsumer added) {
        if (stored == desired) return;
        for (var iterator = stored.iterator(); iterator.hasNext();) {
            long section = iterator.nextLong();
            if (desired.contains(section)) continue;
            iterator.remove();
            removed.accept(section);
        }
        for (var iterator = desired.iterator(); iterator.hasNext();) {
            long section = iterator.nextLong();
            if (stored.add(section)) added.accept(section);
        }
    }
}
