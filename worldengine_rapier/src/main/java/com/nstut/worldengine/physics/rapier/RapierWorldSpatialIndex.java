package com.nstut.worldengine.physics.rapier;

import com.nstut.worldengine.api.PhysicsRegion;
import com.nstut.worldengine.api.WorldSpatialIndex;
import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Fixed, sparse world-region grid. Phase 7 replaces the fixed grouping with
 * interaction-graph merge/split; this layer provides local origins and safe
 * migration without requiring a single dimension-wide Rapier world.
 */
public class RapierWorldSpatialIndex implements WorldSpatialIndex {
    public static final double REGION_SIZE = 4096.0;
    private static final double REGION_HALF_SIZE = REGION_SIZE * 0.5;
    private static final double MIGRATION_HYSTERESIS = 128.0;
    private static final int EMPTY_REGION_RETENTION_TICKS = 200;
    private static final double INTERACTION_CELL_SIZE = 128.0;
    private static final double INTERACTION_MARGIN = 8.0;
    private static final double INTERACTION_HORIZON_SECONDS = 2.0;
    private static final long INTERACTION_SPLIT_DELAY_TICKS = 40;

    private record RegionKey(int x, int y, int z) {}
    private record InteractionCell(int x, int y, int z) {}
    private record InteractionBounds(double minX, double minY, double minZ,
                                     double maxX, double maxY, double maxZ) {
        boolean intersects(InteractionBounds other) {
            return this.minX <= other.maxX && this.maxX >= other.minX
                    && this.minY <= other.maxY && this.maxY >= other.minY
                    && this.minZ <= other.maxZ && this.maxZ >= other.minZ;
        }
    }
    private record Migration(ServerSubLevel subLevel, RapierPhysicsRegion source, RegionKey destination) {}
    private record InteractionExpiry(long expiryTick, int bodyId) implements Comparable<InteractionExpiry> {
        @Override
        public int compareTo(InteractionExpiry o) {
            return Long.compare(this.expiryTick, o.expiryTick);
        }
    }
    private record RegionExpiry(long expiryTick, RapierPhysicsRegion region) implements Comparable<RegionExpiry> {
        @Override
        public int compareTo(RegionExpiry o) {
            return Long.compare(this.expiryTick, o.expiryTick);
        }
    }

    private final List<PhysicsRegion> regions = new ArrayList<>();
    private final Object2ObjectMap<RegionKey, ObjectOpenHashSet<RapierPhysicsRegion>> regionGrid = new Object2ObjectOpenHashMap<>();
    private final Int2ObjectMap<RapierPhysicsRegion> subLevelRegionMap = new Int2ObjectOpenHashMap<>();
    private final Map<RapierPhysicsRegion, Long> emptyRegionExpiry = new HashMap<>();
    private final PriorityQueue<RegionExpiry> emptyRegionQueue = new PriorityQueue<>();
    private final Map<Integer, Long> interactionHoldUntil = new HashMap<>();
    private final PriorityQueue<InteractionExpiry> interactionHoldQueue = new PriorityQueue<>();
    private final Map<InteractionCell, Set<Integer>> interactionCells = new HashMap<>();
    private final Map<Integer, Set<InteractionCell>> bodyInteractionCells = new HashMap<>();
    private final Map<Integer, InteractionBounds> bodyInteractionBounds = new HashMap<>();
    private final Map<Integer, Set<Integer>> interactionEdges = new HashMap<>();
    private final Set<Integer> oversizedInteractionBodies = new HashSet<>();
    private final Set<Integer> dirtyInteractionBodies = new HashSet<>();
    private final RapierPhysicsPipeline pipeline;
    private RapierPhysicsRegion defaultRegion;
    private long currentTick;

    public RapierWorldSpatialIndex(RapierPhysicsPipeline pipeline) {
        this.pipeline = pipeline;
    }

    private static int regionCoordinate(double coordinate) {
        return Math.toIntExact((long) Math.floor((coordinate + REGION_HALF_SIZE) / REGION_SIZE));
    }

    private static RegionKey keyFor(Vector3dc position) {
        return new RegionKey(
                regionCoordinate(position.x()),
                regionCoordinate(position.y()),
                regionCoordinate(position.z()));
    }

    private static int interactionCellCoordinate(double coordinate) {
        return (int) Math.floor(coordinate / INTERACTION_CELL_SIZE);
    }

    private static InteractionBounds interactionBounds(ServerSubLevel subLevel) {
        BoundingBox3dc bounds = subLevel.boundingBox();
        Vector3dc velocity = subLevel.latestLinearVelocity;
        double dx = velocity.x() * INTERACTION_HORIZON_SECONDS;
        double dy = velocity.y() * INTERACTION_HORIZON_SECONDS;
        double dz = velocity.z() * INTERACTION_HORIZON_SECONDS;
        return new InteractionBounds(
                bounds.minX() + Math.min(0.0, dx) - INTERACTION_MARGIN,
                bounds.minY() + Math.min(0.0, dy) - INTERACTION_MARGIN,
                bounds.minZ() + Math.min(0.0, dz) - INTERACTION_MARGIN,
                bounds.maxX() + Math.max(0.0, dx) + INTERACTION_MARGIN,
                bounds.maxY() + Math.max(0.0, dy) + INTERACTION_MARGIN,
                bounds.maxZ() + Math.max(0.0, dz) + INTERACTION_MARGIN);
    }

    private RapierPhysicsRegion createRegion(RegionKey key) {
        Vector3d origin = new Vector3d(key.x * REGION_SIZE, key.y * REGION_SIZE, key.z * REGION_SIZE);
        RapierPhysicsRegion created = new RapierPhysicsRegion(
                this.pipeline, this.pipeline.getGravity(), this.pipeline.getUniversalDrag(), origin);

        this.regionGrid.computeIfAbsent(key, ignored -> new ObjectOpenHashSet<>()).add(created);
        this.regions.add(created);
        this.pipeline.registerRegion(created);
        this.pipeline.populateRegionTerrain(created);
        return created;
    }

    private RapierPhysicsRegion getOrCreateRegion(RegionKey key) {
        ObjectOpenHashSet<RapierPhysicsRegion> existingSet = this.regionGrid.get(key);
        if (existingSet != null) {
            for (RapierPhysicsRegion region : existingSet) {
                if (region != this.defaultRegion) return region;
            }
        }
        return this.createRegion(key);
    }

    public RapierPhysicsRegion getDefaultRegion() {
        if (this.defaultRegion == null) {
            // Dedicated auxiliary scene for boxes, ropes and kinematic objects.
            // ServerSubLevels never share this region.
            this.defaultRegion = this.createRegion(new RegionKey(0, 0, 0));
        }
        return this.defaultRegion;
    }

    @Override
    public void addSubLevel(ServerSubLevel subLevel) {
        // No-op for dormant bodies; spatial index only tracks resident bodies.
    }

    public RapierPhysicsRegion ensureResident(ServerSubLevel body) {
        RapierPhysicsRegion existing = this.getRegion(body);
        if (existing != null) return existing;
        this.pipeline.readPose(body, body.logicalPose());
        this.pipeline.getLinearVelocity(body, (Vector3d) body.latestLinearVelocity);
        body.updateBoundingBox();
        Vector3dc pos = body.logicalPose().position();
        this.pipeline.ensureTerrainNear(pos);
        RapierPhysicsRegion region = this.materializeSubLevel(body, pos);
        Rapier3D.materializeBody(this.pipeline.getUniverseHandle(), Rapier3D.getID(body), region.getSceneHandle());
        this.pipeline.streamRegionTerrain(region);
        this.pipeline.markRegionDirty(region);
        return region;
    }

    public RapierPhysicsRegion materializeSubLevel(ServerSubLevel subLevel, Vector3dc position) {
        RapierPhysicsRegion region = this.getOrCreateRegion(keyFor(position));
        if (region == this.defaultRegion) {
            region = this.createRegion(keyFor(position));
        }
        this.emptyRegionExpiry.remove(region);
        region.addSubLevel(subLevel);
        int id = Rapier3D.getID(subLevel);
        this.subLevelRegionMap.put(id, region);
        this.dirtyInteractionBodies.add(id);
        return region;
    }

    public void evictSubLevel(ServerSubLevel subLevel) {
        int id = Rapier3D.getID(subLevel);
        RapierPhysicsRegion region = this.subLevelRegionMap.remove(id);
        if (region != null) {
            region.removeSubLevel(subLevel);
            this.retainRegionIfEmpty(region);
        }
    }

    @Override
    public void removeSubLevel(ServerSubLevel subLevel) {
        int id = Rapier3D.getID(subLevel);
        this.interactionHoldUntil.remove(id);
        this.removeInteractionBody(id);
        RapierPhysicsRegion region = this.subLevelRegionMap.remove(id);
        if (region != null) {
            region.removeSubLevel(subLevel);
            this.retainRegionIfEmpty(region);
        }
    }

    void retainRegionIfEmpty(RapierPhysicsRegion region) {
        if (region == this.defaultRegion || !region.getActiveSubLevels().isEmpty()) return;
        long expiry = this.currentTick + EMPTY_REGION_RETENTION_TICKS;
        if (this.emptyRegionExpiry.putIfAbsent(region, expiry) == null) {
            this.emptyRegionQueue.add(new RegionExpiry(expiry, region));
        }
    }

    private void disposeRegion(RapierPhysicsRegion region) {
        this.emptyRegionExpiry.remove(region);
        this.regions.remove(region);
        for (ObjectOpenHashSet<RapierPhysicsRegion> set : this.regionGrid.values()) {
            set.remove(region);
        }
        this.regionGrid.values().removeIf(ObjectOpenHashSet::isEmpty);
        this.pipeline.unregisterRegion(region);
        region.dispose();
    }

    @Override
    public Collection<PhysicsRegion> getRegions() {
        return this.regions;
    }

    public RapierPhysicsRegion getRegion(ServerSubLevel subLevel) {
        return this.subLevelRegionMap.get(Rapier3D.getID(subLevel));
    }

    public RapierPhysicsRegion getRegion(int subLevelId) {
        return this.subLevelRegionMap.get(subLevelId);
    }

    public void markBodyMoved(ServerSubLevel subLevel) {
        int id = Rapier3D.getID(subLevel);
        this.dirtyInteractionBodies.add(id);
        RapierPhysicsRegion region = this.subLevelRegionMap.get(id);
        if (region != null) region.markTerrainDirty(id);
    }

    private void removeInteractionBody(int id) {
        Set<InteractionCell> oldCells = this.bodyInteractionCells.remove(id);
        if (oldCells != null) {
            for (InteractionCell cell : oldCells) {
                Set<Integer> members = this.interactionCells.get(cell);
                if (members == null) continue;
                members.remove(id);
                if (members.isEmpty()) this.interactionCells.remove(cell);
            }
        }
        Set<Integer> neighbors = this.interactionEdges.remove(id);
        if (neighbors != null) {
            for (int neighbor : neighbors) {
                Set<Integer> edges = this.interactionEdges.get(neighbor);
                if (edges != null) edges.remove(id);
            }
        }
        this.bodyInteractionBounds.remove(id);
        this.oversizedInteractionBodies.remove(id);
        this.dirtyInteractionBodies.remove(id);
    }

    boolean migrateTo(ServerSubLevel subLevel, RapierPhysicsRegion destination) {
        RapierPhysicsRegion source = this.getRegion(subLevel);
        if (source == null || source == destination) return true;
        int id = Rapier3D.getID(subLevel);
        if (!Rapier3D.migrateBody(source.getSceneHandle(), destination.getSceneHandle(), id)) {
            return false;
        }
        source.removeSubLevel(subLevel);
        this.emptyRegionExpiry.remove(destination);
        destination.addSubLevel(subLevel);
        this.subLevelRegionMap.put(id, destination);
        this.retainRegionIfEmpty(source);
        this.pipeline.streamRegionTerrain(source);
        this.pipeline.streamRegionTerrain(destination);
        this.pipeline.markRegionDirty(source);
        this.pipeline.markRegionDirty(destination);
        return true;
    }

    private void rebaseRegionTo(RapierPhysicsRegion region, RegionKey targetKey) {
        if (region == this.defaultRegion) return;
        RegionKey oldKey = keyFor(region.getOrigin());
        ObjectOpenHashSet<RapierPhysicsRegion> oldSet = this.regionGrid.get(oldKey);
        if (oldSet != null) {
            oldSet.remove(region);
            if (oldSet.isEmpty()) this.regionGrid.remove(oldKey);
        }
        Vector3d newOrigin = new Vector3d(targetKey.x * REGION_SIZE, targetKey.y * REGION_SIZE, targetKey.z * REGION_SIZE);
        region.rebaseOrigin(newOrigin);
        this.regionGrid.computeIfAbsent(targetKey, k -> new ObjectOpenHashSet<>()).add(region);
        this.pipeline.streamRegionTerrain(region);
        this.pipeline.markRegionDirty(region);
    }

    private static boolean outsideMigrationBoundary(RapierPhysicsRegion region, Vector3dc position) {
        Vector3dc origin = region.getOrigin();
        double limit = REGION_HALF_SIZE + MIGRATION_HYSTERESIS;
        return Math.abs(position.x() - origin.x()) > limit
                || Math.abs(position.y() - origin.y()) > limit
                || Math.abs(position.z() - origin.z()) > limit;
    }

    private Set<InteractionCell> cellsFor(InteractionBounds bounds) {
        int minX = interactionCellCoordinate(bounds.minX);
        int minY = interactionCellCoordinate(bounds.minY);
        int minZ = interactionCellCoordinate(bounds.minZ);
        int maxX = interactionCellCoordinate(bounds.maxX);
        int maxY = interactionCellCoordinate(bounds.maxY);
        int maxZ = interactionCellCoordinate(bounds.maxZ);
        long count = (long) (maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1);
        if (count > 4096) return Set.of();
        Set<InteractionCell> result = new HashSet<>((int) count);
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) result.add(new InteractionCell(x, y, z));
            }
        }
        return result;
    }

    private void updateInteractionGraph(Set<Integer> movedBodies) {
        Set<Integer> affected = new HashSet<>();
        for (int id : movedBodies) {
            RapierPhysicsRegion region = this.subLevelRegionMap.get(id);
            ServerSubLevel body = region == null ? null : region.getSubLevel(id);
            if (body == null || body.isRemoved()) {
                this.removeInteractionBody(id);
                continue;
            }

            Set<Integer> previous = new HashSet<>(this.interactionEdges.getOrDefault(id, Set.of()));
            affected.add(id);
            affected.addAll(previous);
            Set<InteractionCell> oldCells = this.bodyInteractionCells.remove(id);
            if (oldCells != null) {
                for (InteractionCell cell : oldCells) {
                    Set<Integer> members = this.interactionCells.get(cell);
                    if (members != null) {
                        members.remove(id);
                        if (members.isEmpty()) this.interactionCells.remove(cell);
                    }
                }
            }

            InteractionBounds bounds = interactionBounds(body);
            Set<InteractionCell> cells = this.cellsFor(bounds);
            this.bodyInteractionBounds.put(id, bounds);
            this.bodyInteractionCells.put(id, cells);
            if (cells.isEmpty()) this.oversizedInteractionBodies.add(id);
            else this.oversizedInteractionBodies.remove(id);

            Set<Integer> candidates = new HashSet<>(this.oversizedInteractionBodies);
            if (cells.isEmpty()) candidates.addAll(this.bodyInteractionBounds.keySet());
            for (InteractionCell cell : cells) {
                candidates.addAll(this.interactionCells.getOrDefault(cell, Set.of()));
                this.interactionCells.computeIfAbsent(cell, ignored -> new HashSet<>()).add(id);
            }
            candidates.remove(id);
            Set<Integer> current = new HashSet<>();
            for (int candidate : candidates) {
                InteractionBounds candidateBounds = this.bodyInteractionBounds.get(candidate);
                if (candidateBounds != null && bounds.intersects(candidateBounds)) current.add(candidate);
            }

            for (int removed : previous) {
                if (current.contains(removed)) continue;
                Set<Integer> edges = this.interactionEdges.get(removed);
                if (edges != null) edges.remove(id);
            }
            this.interactionEdges.put(id, current);
            for (int added : current) {
                this.interactionEdges.computeIfAbsent(added, ignored -> new HashSet<>()).add(id);
            }
            affected.addAll(current);
        }

        Set<Integer> visited = new HashSet<>();
        for (int seed : affected) {
            if (!visited.add(seed)) continue;
            List<ServerSubLevel> component = new ArrayList<>();
            ArrayDeque<Integer> pending = new ArrayDeque<>();
            pending.add(seed);
            while (!pending.isEmpty()) {
                int id = pending.removeFirst();
                RapierPhysicsRegion region = this.subLevelRegionMap.get(id);
                ServerSubLevel body = region == null ? null : region.getSubLevel(id);
                if (body != null && !body.isRemoved()) component.add(body);
                for (int neighbor : this.interactionEdges.getOrDefault(id, Set.of())) {
                    if (visited.add(neighbor)) pending.addLast(neighbor);
                }
            }
            if (component.size() < 2) continue;
            RapierPhysicsRegion target = this.getRegion(component.getFirst());
            if (target == null) continue;
            for (ServerSubLevel body : component) {
                long expiry = this.currentTick + INTERACTION_SPLIT_DELAY_TICKS;
                this.interactionHoldUntil.put(Rapier3D.getID(body), expiry);
                this.interactionHoldQueue.add(new InteractionExpiry(expiry, Rapier3D.getID(body)));
                RapierPhysicsRegion source = this.getRegion(body);
                if (source != null && source != target && !this.migrateTo(body, target)) {
                    if (!this.mergeRegions(source, target)) {
                        throw new IllegalStateException(
                                "Unable to merge interacting constrained physics regions");
                    }
                }
            }
        }
    }

    public void reconcileDirtyInteractions() {
        if (this.dirtyInteractionBodies.isEmpty() && this.interactionHoldQueue.isEmpty()) {
            return;
        }
        Set<Integer> movedBodies = new HashSet<>(this.dirtyInteractionBodies);
        this.dirtyInteractionBodies.clear();
        while (!this.interactionHoldQueue.isEmpty() && this.interactionHoldQueue.peek().expiryTick() <= this.currentTick) {
            InteractionExpiry entry = this.interactionHoldQueue.poll();
            Long currentExpiry = this.interactionHoldUntil.get(entry.bodyId());
            if (currentExpiry != null && currentExpiry == entry.expiryTick()) {
                this.interactionHoldUntil.remove(entry.bodyId());
                movedBodies.add(entry.bodyId());
            }
        }
        this.updateInteractionGraph(movedBodies);
        List<Migration> migrations = new ArrayList<>();
        for (int id : movedBodies) {
            RapierPhysicsRegion source = this.subLevelRegionMap.get(id);
            if (source == null) continue;
            ServerSubLevel subLevel = source.getSubLevel(id);
            if (subLevel == null || subLevel.isRemoved()) continue;
            if (this.interactionHoldUntil.getOrDefault(id, 0L) > this.currentTick) continue;
            Vector3dc position = subLevel.logicalPose().position();
            if (!outsideMigrationBoundary(source, position)) continue;
            RegionKey targetKey = keyFor(position);
            migrations.add(new Migration(subLevel, source, targetKey));
        }

        for (Migration migration : migrations) {
            ObjectOpenHashSet<RapierPhysicsRegion> existingRegions =
                    this.regionGrid.get(migration.destination());
            boolean migrated = false;

            if (existingRegions != null && !existingRegions.isEmpty()) {
                for (RapierPhysicsRegion region : existingRegions) {
                    if (region == migration.source() || region == this.defaultRegion) continue;
                    if (this.migrateTo(migration.subLevel(), region)) {
                        migrated = true;
                        break;
                    }
                }
            }

            if (!migrated && migration.source() != this.defaultRegion) {
                // Keep a joint-pinned interaction cluster in one Rapier scene.
                // Multiple independent scenes may occupy the same macro-cell.
                this.rebaseRegionTo(migration.source(), migration.destination());
            }
        }
    }

    public void advanceLifecycleTimers() {
        while (!this.emptyRegionQueue.isEmpty() && this.emptyRegionQueue.peek().expiryTick() <= this.currentTick) {
            RegionExpiry entry = this.emptyRegionQueue.poll();
            Long currentExpiry = this.emptyRegionExpiry.get(entry.region());
            if (currentExpiry != null && currentExpiry == entry.expiryTick()) {
                this.emptyRegionExpiry.remove(entry.region());
                this.disposeRegion(entry.region());
            }
        }
    }

    @Override
    public void tick() {
        this.currentTick++;
        this.reconcileDirtyInteractions();
        this.advanceLifecycleTimers();
    }

    boolean mergeRegions(RapierPhysicsRegion source, RapierPhysicsRegion destination) {
        if (source == destination) return true;
        if (source == this.defaultRegion || destination == this.defaultRegion) return false;
        if (!Rapier3D.mergeScenes(source.getSceneHandle(), destination.getSceneHandle())) return false;

        for (ServerSubLevel subLevel : new ArrayList<>(source.getActiveSubLevels())) {
            source.removeSubLevel(subLevel);
            destination.addSubLevel(subLevel);
            int id = Rapier3D.getID(subLevel);
            this.subLevelRegionMap.put(id, destination);
            this.dirtyInteractionBodies.add(id);
            destination.markTerrainDirty(id);
        }

        this.pipeline.streamRegionTerrain(destination);
        this.pipeline.markRegionDirty(destination);
        this.disposeRegion(source);
        return true;
    }

    @Override
    public void dispose() {
        for (PhysicsRegion region : this.regions) {
            region.dispose();
        }
        this.regions.clear();
        this.regionGrid.clear();
        this.subLevelRegionMap.clear();
        this.emptyRegionExpiry.clear();
        this.emptyRegionQueue.clear();
        this.interactionHoldUntil.clear();
        this.interactionHoldQueue.clear();
        this.interactionCells.clear();
        this.bodyInteractionCells.clear();
        this.bodyInteractionBounds.clear();
        this.interactionEdges.clear();
        this.oversizedInteractionBodies.clear();
        this.dirtyInteractionBodies.clear();
        this.defaultRegion = null;
    }
}
