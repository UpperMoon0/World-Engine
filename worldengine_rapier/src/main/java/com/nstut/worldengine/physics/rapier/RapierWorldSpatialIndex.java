package com.nstut.worldengine.physics.rapier;

import com.nstut.worldengine.api.PhysicsRegion;
import com.nstut.worldengine.api.WorldSpatialIndex;
import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.ints.IntSet;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

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
    private static final double INTERACTION_MARGIN = 8.0;
    private static final double INTERACTION_HORIZON_SECONDS = 2.0;
    private static final long INTERACTION_SPLIT_DELAY_TICKS = 40;

    private record RegionKey(int x, int y, int z) {}
    private record Migration(ServerSubLevel subLevel, RapierPhysicsRegion source, RegionKey destination) {}
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
    private final InteractionHoldTracker interactionHolds = new InteractionHoldTracker();
    private final InteractionGraph interactionGraph = new InteractionGraph();
    private final IntSet dirtyInteractionBodies = new IntOpenHashSet();
    private final IntSet affectedInteractionBodies = new IntOpenHashSet();
    private final IntSet visitedInteractionBodies = new IntOpenHashSet();
    // Reconciliation runs on the server thread. Migration/merge paths enqueue
    // dirty bodies for the next pass and never recursively reconcile this index.
    private final IntSet movedBodiesScratch = new IntOpenHashSet();
    private final List<ServerSubLevel> componentScratch = new ArrayList<>();
    private final List<Migration> migrationsScratch = new ArrayList<>();
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

    private void updateInteractionBounds(int id, ServerSubLevel subLevel, IntSet affected) {
        BoundingBox3dc bounds = subLevel.boundingBox();
        Vector3dc velocity = subLevel.latestLinearVelocity;
        double dx = velocity.x() * INTERACTION_HORIZON_SECONDS;
        double dy = velocity.y() * INTERACTION_HORIZON_SECONDS;
        double dz = velocity.z() * INTERACTION_HORIZON_SECONDS;
        this.interactionGraph.update(id,
                bounds.minX() + Math.min(0.0, dx) - INTERACTION_MARGIN,
                bounds.minY() + Math.min(0.0, dy) - INTERACTION_MARGIN,
                bounds.minZ() + Math.min(0.0, dz) - INTERACTION_MARGIN,
                bounds.maxX() + Math.max(0.0, dx) + INTERACTION_MARGIN,
                bounds.maxY() + Math.max(0.0, dy) + INTERACTION_MARGIN,
                bounds.maxZ() + Math.max(0.0, dz) + INTERACTION_MARGIN, affected);
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

    RapierPhysicsRegion regionForHandle(long handle) {
        for (PhysicsRegion region : this.regions) {
            if (region.getSceneHandle() == handle) return (RapierPhysicsRegion) region;
        }
        throw new IllegalStateException("Rope scene is no longer registered");
    }

    public RapierPhysicsRegion getDefaultRegion() {
        if (this.defaultRegion == null) {
            // Dedicated auxiliary scene for boxes, ropes and kinematic objects.
            // Ropes move to a sublevel region when attached to a body.
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
        this.interactionHolds.remove(id);
        this.removeInteractionBody(id);
        RapierPhysicsRegion region = this.subLevelRegionMap.remove(id);
        if (region != null) {
            region.removeSubLevel(subLevel);
            this.retainRegionIfEmpty(region);
        }
    }

    void retainRegionIfEmpty(RapierPhysicsRegion region) {
        if (region == this.defaultRegion || !region.getActiveSubLevels().isEmpty() || this.pipeline.hasRopes(region)) return;
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
        this.interactionGraph.remove(id);
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

    private void updateInteractionGraph(IntSet movedBodies) {
        IntSet affected = this.affectedInteractionBodies;
        affected.clear();
        for (var idIterator = movedBodies.iterator(); idIterator.hasNext();) {
            int id = idIterator.nextInt();
            RapierPhysicsRegion region = this.subLevelRegionMap.get(id);
            ServerSubLevel body = region == null ? null : region.getSubLevel(id);
            if (body == null || body.isRemoved()) {
                this.removeInteractionBody(id);
                continue;
            }
            this.updateInteractionBounds(id, body, affected);
        }

        IntSet visited = this.visitedInteractionBodies;
        visited.clear();
        for (var seedIterator = affected.iterator(); seedIterator.hasNext();) {
            int seed = seedIterator.nextInt();
            if (!visited.add(seed)) continue;
            List<ServerSubLevel> component = this.componentScratch;
            component.clear();
            for (int id : this.interactionGraph.component(seed)) {
                visited.add(id);
                RapierPhysicsRegion region = this.subLevelRegionMap.get(id);
                ServerSubLevel body = region == null ? null : region.getSubLevel(id);
                if (body != null && !body.isRemoved()) component.add(body);
            }
            if (component.size() < 2) continue;
            RapierPhysicsRegion target = this.getRegion(component.getFirst());
            // Keep peers in the local scene containing a body-attached rope.
            for (ServerSubLevel body : component) {
                if (this.pipeline.hasRopes(this.getRegion(body))) {
                    target = this.getRegion(body);
                    break;
                }
            }
            if (target == null) continue;
            for (ServerSubLevel body : component) {
                long expiry = this.currentTick + INTERACTION_SPLIT_DELAY_TICKS;
                this.interactionHolds.renew(Rapier3D.getID(body), expiry);
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
        if (this.dirtyInteractionBodies.isEmpty() && this.interactionHolds.isEmpty()) {
            return;
        }
        IntSet movedBodies = this.movedBodiesScratch;
        movedBodies.clear();
        movedBodies.addAll(this.dirtyInteractionBodies);
        this.dirtyInteractionBodies.clear();
        this.interactionHolds.drainExpired(this.currentTick, movedBodies);
        this.updateInteractionGraph(movedBodies);
        this.interactionHolds.finishExpiryReconciliation();
        List<Migration> migrations = this.migrationsScratch;
        migrations.clear();
        for (var idIterator = movedBodies.iterator(); idIterator.hasNext();) {
            int id = idIterator.nextInt();
            RapierPhysicsRegion source = this.subLevelRegionMap.get(id);
            if (source == null) continue;
            ServerSubLevel subLevel = source.getSubLevel(id);
            if (subLevel == null || subLevel.isRemoved()) continue;
            if (this.interactionHolds.holds(id, this.currentTick)) continue;
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
                if (!this.pipeline.hasRopes(entry.region())) this.disposeRegion(entry.region());
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
        this.pipeline.moveRopes(source, destination);

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
        this.interactionHolds.clear();
        this.interactionGraph.clear();
        this.affectedInteractionBodies.clear();
        this.visitedInteractionBodies.clear();
        this.dirtyInteractionBodies.clear();
        this.movedBodiesScratch.clear();
        this.componentScratch.clear();
        this.migrationsScratch.clear();
        this.defaultRegion = null;
    }
}
