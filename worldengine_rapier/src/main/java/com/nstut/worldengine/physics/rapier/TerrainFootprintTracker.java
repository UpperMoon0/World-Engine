package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import it.unimi.dsi.fastutil.ints.IntSet;

final class TerrainFootprintTracker {
    record Envelope(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
        private static final Envelope EMPTY = new Envelope(0, 0, 0, -1, -1, -1);

        static Envelope fromWorldBounds(double minX, double minY, double minZ,
                                        double maxX, double maxY, double maxZ) {
            if (!Double.isFinite(minX) || !Double.isFinite(minY) || !Double.isFinite(minZ)
                    || !Double.isFinite(maxX) || !Double.isFinite(maxY) || !Double.isFinite(maxZ)) {
                return EMPTY;
            }

            if (minX > 30_000_000.0 || maxX < -30_000_000.0
                    || minZ > 30_000_000.0 || maxZ < -30_000_000.0
                    || minY > 2048.0 || maxY < -2048.0) {
                return EMPTY;
            }

            double clampedMinX = Math.max(-30_000_000.0, minX);
            double clampedMaxX = Math.min(30_000_000.0, maxX);
            double clampedMinY = Math.max(-2048.0, minY);
            double clampedMaxY = Math.min(2048.0, maxY);
            double clampedMinZ = Math.max(-30_000_000.0, minZ);
            double clampedMaxZ = Math.min(30_000_000.0, maxZ);

            int sectionMinX = ((int) Math.floor(clampedMinX)) >> 4;
            int sectionMinY = ((int) Math.floor(clampedMinY)) >> 4;
            int sectionMinZ = ((int) Math.floor(clampedMinZ)) >> 4;
            int sectionMaxX = ((int) Math.floor(clampedMaxX)) >> 4;
            int sectionMaxY = ((int) Math.floor(clampedMaxY)) >> 4;
            int sectionMaxZ = ((int) Math.floor(clampedMaxZ)) >> 4;
            long sizeX = (long) sectionMaxX - sectionMinX + 1L;
            long sizeY = (long) sectionMaxY - sectionMinY + 1L;
            long sizeZ = (long) sectionMaxZ - sectionMinZ + 1L;
            if (sizeX <= 0L || sizeY <= 0L || sizeZ <= 0L
                    || sizeX > 4096L
                    || sizeY > 4096L / sizeX
                    || sizeZ > 4096L / (sizeX * sizeY)) {
                return EMPTY;
            }
            return new Envelope(sectionMinX, sectionMinY, sectionMinZ,
                    sectionMaxX, sectionMaxY, sectionMaxZ);
        }

        boolean isEmpty() {
            return this.maxX < this.minX || this.maxY < this.minY || this.maxZ < this.minZ;
        }
    }

    private final Int2ObjectMap<Envelope> envelopes = new Int2ObjectOpenHashMap<>();
    private final IntSet dirtyBodies = new IntOpenHashSet();

    void markDirty(int id) {
        this.dirtyBodies.add(id);
    }

    void forceDirty(int id) {
        this.envelopes.remove(id);
        this.dirtyBodies.add(id);
    }

    boolean needsRefresh(int id, Envelope envelope) {
        Envelope previous = this.envelopes.put(id, envelope);
        return !envelope.equals(previous);
    }

    int[] drainDirtyBodies() {
        int[] result = this.dirtyBodies.toIntArray();
        this.dirtyBodies.clear();
        return result;
    }

    void remove(int id) {
        this.envelopes.remove(id);
        this.dirtyBodies.remove(id);
    }

    void reset(int[] activeIds) {
        this.envelopes.clear();
        this.dirtyBodies.clear();
        for (int id : activeIds) {
            this.dirtyBodies.add(id);
        }
    }
}
