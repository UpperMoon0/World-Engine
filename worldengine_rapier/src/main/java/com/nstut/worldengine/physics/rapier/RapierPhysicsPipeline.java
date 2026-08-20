package com.nstut.worldengine.physics.rapier;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import com.nstut.worldengine.api.PhysicsRegion;
import com.nstut.worldengine.api.WorldEnginePhysicsSystem;
import com.nstut.worldengine.api.WorldEnginePoseSynchronizer;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.*;
import dev.ryanhcode.sable.api.physics.mass.MassTracker;
import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.api.physics.object.box.BoxPhysicsObject;
import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopePhysicsObject;
import dev.ryanhcode.sable.api.sublevel.KinematicContraption;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.*;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import com.nstut.worldengine.physics.rapier.box.RapierBoxHandle;
import com.nstut.worldengine.physics.rapier.collider.RapierVoxelColliderBakery;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import com.nstut.worldengine.physics.rapier.collider.RapierVoxelColliderData;
import com.nstut.worldengine.physics.rapier.constraint.fixed.RapierFixedConstraintHandle;
import com.nstut.worldengine.physics.rapier.constraint.free.RapierFreeConstraintHandle;
import com.nstut.worldengine.physics.rapier.constraint.generic.RapierGenericConstraintHandle;
import com.nstut.worldengine.physics.rapier.constraint.rotary.RapierRotaryConstraintHandle;
import com.nstut.worldengine.physics.rapier.rope.RapierRopeHandle;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.plot.LevelPlot;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.util.LevelAccelerator;
import dev.ryanhcode.sable.util.SableMathUtils;
import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.longs.LongSet;
import it.unimi.dsi.fastutil.objects.Object2ObjectMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceList;
import it.unimi.dsi.fastutil.objects.ReferenceOpenHashSet;
import net.minecraft.CrashReport;
import net.minecraft.CrashReportCategory;
import net.minecraft.ReportedException;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.core.SectionPos;
import net.minecraft.core.particles.BlockParticleOption;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.sounds.SoundSource;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.chunk.LevelChunkSection;
import net.minecraft.world.phys.Vec3;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Implementation of {@link PhysicsPipeline} for the rust Rapier 3D physics engine.
 */
public class RapierPhysicsPipeline implements PhysicsPipeline, WorldEnginePoseSynchronizer {
    private record ScheduledRegion(long tick, long generation, RapierPhysicsRegion region) {}
    private record RegionStep(RapierPhysicsRegion region, int elapsedTicks) {}

    /**
     * Distance threshold for uploading sub-contraptions to the physics pipeline
     */
    private static final double DISTANCE_THRESHOLD = 1e-7;

    /**
     * Angle threshold for uploading sub-contraptions to the physics pipeline
     */
    private static final double ANGULAR_THRESHOLD = 1e-7;

    private final ServerLevel level;
    private final LevelAccelerator accelerator;
    private final RapierVoxelColliderBakery colliderBakery;
    private final Int2ObjectMap<ServerSubLevel> registeredSubLevels = new Int2ObjectOpenHashMap<>();
    private final Object2ObjectMap<KinematicContraption, TrackedKinematicContraption> activeContraptions = new Object2ObjectOpenHashMap<>();
    private final List<BoxPhysicsObject> activeBoxes = new ArrayList<>();
    private final List<RopePhysicsObject> activeRopes = new ArrayList<>();
    private final Long2LongOpenHashMap recentCollisions = new Long2LongOpenHashMap();
    private final Long2ObjectMap<int[]> globalChunkCache = new Long2ObjectOpenHashMap<>();
    private final Long2ObjectMap<ReferenceOpenHashSet<RapierPhysicsRegion>> terrainSectionRegions = new Long2ObjectOpenHashMap<>();
    private final ReferenceList<PhysicsPipelineBody> queuedWakeUps = new ReferenceArrayList<>();
    private final Set<RapierPhysicsRegion> activeRegions = new ReferenceOpenHashSet<>();
    private final Set<RapierPhysicsRegion> dirtyRegions = new ReferenceOpenHashSet<>();
    private final Set<RapierPhysicsRegion> steppedRegions = new ReferenceOpenHashSet<>();
    private final PriorityQueue<ScheduledRegion> scheduledRegions = new PriorityQueue<>(Comparator.comparingLong(ScheduledRegion::tick));
    private final ExecutorService regionWorkers;
    private final double[] poseCache;
    private ByteBuffer materializationBuffer = ByteBuffer.allocateDirect(1024 * 32).order(ByteOrder.nativeOrder());
    private RapierWorldSpatialIndex spatialIndex;
    private Vector3dc gravity;
    private double universalDrag;
    private long physicsTickCounter;
    private long universeHandle;

    public long getUniverseHandle() { return this.universeHandle; }

    public Vector3dc getGravity() { return this.gravity; }
    public double getUniversalDrag() { return this.universalDrag; }

    void registerRegion(RapierPhysicsRegion region) {
        this.markRegionDirty(region);
    }

    void unregisterRegion(RapierPhysicsRegion region) {
        this.activeRegions.remove(region);
        this.dirtyRegions.remove(region);
        this.steppedRegions.remove(region);
        region.invalidateSchedule();
        for (long key : region.getRequiredTerrainSections().toLongArray()) {
            this.removeTerrainInterest(region, key);
        }
    }

    void markRegionDirty(RapierPhysicsRegion region) {
        region.invalidateSchedule();
        this.dirtyRegions.add(region);
    }

    void addTerrainInterest(RapierPhysicsRegion region, long section) {
        this.terrainSectionRegions.computeIfAbsent(section, ignored -> new ReferenceOpenHashSet<>()).add(region);
    }

    void removeTerrainInterest(RapierPhysicsRegion region, long section) {
        ReferenceOpenHashSet<RapierPhysicsRegion> regions = this.terrainSectionRegions.get(section);
        if (regions == null) return;
        regions.remove(region);
        if (regions.isEmpty()) this.terrainSectionRegions.remove(section);
    }

    private Iterable<RapierPhysicsRegion> terrainRegions(long section) {
        ReferenceOpenHashSet<RapierPhysicsRegion> regions = this.terrainSectionRegions.get(section);
        return regions == null ? List.of() : regions;
    }

    public RapierPhysicsPipeline(final ServerLevel level) {
        this.level = level;
        this.accelerator = new LevelAccelerator(level);
        this.colliderBakery = new RapierVoxelColliderBakery(this.accelerator);
        this.recentCollisions.defaultReturnValue(-1);
        this.poseCache = new double[7];
        int workerCount = Math.max(1, Runtime.getRuntime().availableProcessors() - 1);
        this.regionWorkers = Executors.newFixedThreadPool(workerCount, runnable -> {
            Thread thread = new Thread(runnable, "Sable-Rapier-Region");
            thread.setDaemon(true);
            return thread;
        });
    }

    /**
     * Packs a voxel collider ID and neighborhood state into an integer the rapier companion library will re-interpret as a block-state.
     * @return the packed block state
     */
    private static int packBlockState(final VoxelNeighborhoodState state, final int colliderID) {
        return ((int) state.byteRepresentation()) | (colliderID << 16);
    }

    protected RapierPhysicsRegion getRegion(PhysicsPipelineBody body) {
        if (this.spatialIndex == null) {
            throw new IllegalStateException("Physics scene is not initialized");
        }
        if (body instanceof ServerSubLevel subLevel) {
            return this.spatialIndex.getRegion(subLevel);
        }
        return (RapierPhysicsRegion) this.spatialIndex.getDefaultRegion();
    }

    protected long getSceneHandle(PhysicsPipelineBody body) {
        RapierPhysicsRegion region = getRegion(body);
        return region == null ? 0 : region.getSceneHandle();
    }
    
    protected long getDefaultSceneHandle() {
        if (this.spatialIndex == null) {
            throw new IllegalStateException("Physics scene is not initialized");
        }
        return this.spatialIndex.getDefaultRegion().getSceneHandle();
    }

    public long prepareConstraintScene(@Nullable PhysicsPipelineBody bodyA, @Nullable PhysicsPipelineBody bodyB) {
        RapierPhysicsRegion regionA = bodyA instanceof ServerSubLevel subA
                ? this.spatialIndex.ensureResident(subA)
                : (bodyA == null ? null : this.getRegion(bodyA));
        RapierPhysicsRegion regionB = bodyB instanceof ServerSubLevel subB
                ? this.spatialIndex.ensureResident(subB)
                : (bodyB == null ? null : this.getRegion(bodyB));
        RapierPhysicsRegion target = regionA != null ? regionA : (regionB != null ? regionB : this.spatialIndex.getDefaultRegion());

        if (regionA != null && regionB != null && regionA != regionB) {
            if (!(bodyB instanceof ServerSubLevel subLevel)) {
                throw new IllegalStateException(
                        "Cannot create a cross-region constraint for a non-sublevel body");
            }
            if (!this.spatialIndex.migrateTo(subLevel, target)
                    && !this.spatialIndex.mergeRegions(regionB, target)) {
                throw new IllegalStateException(
                        "Cannot coalesce regions for a cross-region constraint");
            }
        }
        return target.getSceneHandle();
    }

    /**
     * Initializes the physics pipeline.
     *
     * @param gravity the gravity vector
     * @param universalDrag the universal drag to apply to all bodies
     */
    @Override
    public void init(@Nullable final Vector3dc gravity, final double universalDrag) {
        this.gravity = gravity;
        this.universalDrag = universalDrag;
        this.spatialIndex = new RapierWorldSpatialIndex(this);
        this.universeHandle = Rapier3D.createUniverse(0);
        // Native configuration and voxel-collider registration can run as soon
        // as pipeline initialization returns. Create the default scene eagerly
        // so Rapier's process-global state exists before those JNI calls.
        this.spatialIndex.getDefaultRegion();
    }

    /**
     * Disposes all resources used by the physics pipeline.
     */
    @Override
    public void dispose() {
        if (this.spatialIndex != null) {
            this.spatialIndex.dispose();
            this.spatialIndex = null;
        }
        this.activeRegions.clear();
        this.dirtyRegions.clear();
        this.steppedRegions.clear();
        this.scheduledRegions.clear();
        this.terrainSectionRegions.clear();
        this.regionWorkers.shutdown();
        if (this.universeHandle != 0) {
            Rapier3D.destroyUniverse(this.universeHandle);
            this.universeHandle = 0;
        }
    }

    /**
     * Runs a physics tick with a time step of {@code 1.0 / 20.0} seconds.
     */
    @Override
    public void prePhysicsTicks() {
        if (this.universeHandle == 0) return;
        Rapier3D.tickUniverse(this.universeHandle, this.physicsTickCounter, 1.0 / 20.0,
                this.gravity.x(), this.gravity.y(), this.gravity.z());

        int capacityEntries = this.materializationBuffer.capacity() / 32;
        int count = Rapier3D.writeMaterializationRequests(this.universeHandle, this.materializationBuffer, capacityEntries);
        if (count < 0) {
            int requiredCount = -count;
            this.materializationBuffer = ByteBuffer.allocateDirect(requiredCount * 32).order(ByteOrder.nativeOrder());
            count = Rapier3D.writeMaterializationRequests(this.universeHandle, this.materializationBuffer, requiredCount);
        }

        if (count > 0) {
            this.materializationBuffer.position(0);
            List<ServerSubLevel> newlyMaterialized = new ArrayList<>(count);
            for (int i = 0; i < count; i++) {
                int id = this.materializationBuffer.getInt();
                this.materializationBuffer.getInt(); // padding
                double x = this.materializationBuffer.getDouble();
                double y = this.materializationBuffer.getDouble();
                double z = this.materializationBuffer.getDouble();

                ServerSubLevel subLevel = this.registeredSubLevels.get(id);
                if (subLevel != null && !subLevel.isRemoved()) {
                    Vector3dc pos = new org.joml.Vector3d(x, y, z);
                    this.ensureTerrainNear(pos);
                    RapierPhysicsRegion region = this.spatialIndex.materializeSubLevel(subLevel, pos);
                    Rapier3D.materializeBody(this.universeHandle, id, region.getSceneHandle());
                    this.streamRegionTerrain(region);
                    this.markRegionDirty(region);
                    newlyMaterialized.add(subLevel);
                }
            }
            for (ServerSubLevel subLevel : newlyMaterialized) {
                this.readPose(subLevel, subLevel.logicalPose());
                this.getLinearVelocity(subLevel, (Vector3d) subLevel.latestLinearVelocity);
                subLevel.updateBoundingBox();
                this.spatialIndex.markBodyMoved(subLevel);
                RapierPhysicsRegion region = this.spatialIndex.getRegion(subLevel);
                if (region != null) {
                    this.streamRegionTerrain(region);
                }
            }
            this.spatialIndex.reconcileDirtyInteractions();
        }

        this.drainUniverseEvictions();
        Rapier3D.flushUniverseCommands(this.universeHandle);
    }

    public void drainUniverseEvictions() {
        if (this.universeHandle == 0) return;
        int[] evictions = Rapier3D.drainEvictionEvents(this.universeHandle);
        if (evictions != null && evictions.length > 0) {
            for (int id : evictions) {
                ServerSubLevel subLevel = this.registeredSubLevels.get(id);
                if (subLevel != null) {
                    this.spatialIndex.evictSubLevel(subLevel);
                }
            }
        }
    }

    /**
     * Runs a physics substep with a time step of {@code 1.0 / 20.0 / substeps} seconds.
     *
     * @param timeStep the time step of this physics substep [s]
     */
    @Override
    public void physicsTick(final double timeStep) {
        this.physicsTickCounter++;
        this.updateContraptionPoses();

        Set<RapierPhysicsRegion> workRegions = new ReferenceOpenHashSet<>(this.activeRegions);
        workRegions.addAll(this.dirtyRegions);
        this.dirtyRegions.clear();
        while (!this.scheduledRegions.isEmpty() && this.scheduledRegions.peek().tick() <= this.physicsTickCounter) {
            ScheduledRegion scheduled = this.scheduledRegions.remove();
            if (scheduled.region().scheduleGeneration() == scheduled.generation()) {
                workRegions.add(scheduled.region());
            }
        }
        if (!this.activeContraptions.isEmpty() || !this.activeBoxes.isEmpty() || !this.activeRopes.isEmpty()) {
            workRegions.add((RapierPhysicsRegion) this.spatialIndex.getDefaultRegion());
        }

        this.steppedRegions.clear();
        List<RegionStep> parallelRegions = new ArrayList<>();
        for (RapierPhysicsRegion region : workRegions) {
            // Bounds and block changes are finalized on the server thread before
            // physics observers tick. Apply their terrain footprints before the
            // region's first step so a newly assembled body never simulates
            // against an empty world scene.
            this.streamRegionTerrain(region);
            region.flushCommands();
            int elapsedTicks = (int) Math.min(Integer.MAX_VALUE,
                    Math.max(1L, this.physicsTickCounter - region.lastStepTick()));
            RegionStep step = new RegionStep(region, elapsedTicks);
            if (region.canStepInParallel()) {
                parallelRegions.add(step);
            } else {
                region.step(timeStep, elapsedTicks);
            }
            region.setLastStepTick(this.physicsTickCounter);
            this.steppedRegions.add(region);
        }

        if (parallelRegions.size() == 1) {
            RegionStep step = parallelRegions.getFirst();
            step.region().step(timeStep, step.elapsedTicks());
        } else if (!parallelRegions.isEmpty()) {
            CompletableFuture<?>[] steps = parallelRegions.stream()
                    .map(step -> CompletableFuture.runAsync(
                            () -> step.region().step(timeStep, step.elapsedTicks()), this.regionWorkers))
                    .toArray(CompletableFuture[]::new);
            CompletableFuture.allOf(steps).join();
        }

        for (final PhysicsPipelineBody queuedWakeUp : this.queuedWakeUps) {
            if (queuedWakeUp.isRemoved()) {
                continue;
            }

            if (queuedWakeUp instanceof ServerSubLevel subLevel) {
                Rapier3D.wakeUpUniverse(this.universeHandle, queuedWakeUp.getRuntimeId());
                RapierPhysicsRegion region = this.getRegion(subLevel);
                if (region != null) {
                    this.markRegionDirty(region);
                }
            }
        }

        this.queuedWakeUps.clear();
        this.drainUniverseEvictions();
    }

    /**
     * Called after all physics substeps have been run, to finalize the physics tick.
     */
    @Override
    public void postPhysicsTicks() {
        this.processCollisionEffects();
    }

    /**
     * Runs a tick to update any separate sub-level tracking / logic, even if physics is currently paused
     */
    @Override
    public void tick() {
        this.accelerator.clearCache();
    }

    /**
     * Adds a {@link SubLevel} to the physics pipeline.
     */
    @Override
    public void add(final ServerSubLevel subLevel, final Pose3dc pose) {
        this.assertBodyValid(subLevel);
        final Vector3dc pos = pose.position();
        final Quaterniondc rot = pose.orientation();

        final int id = Rapier3D.getID(subLevel);
        this.registeredSubLevels.put(id, subLevel);
        Rapier3D.registerUniverseBody(this.universeHandle, id, new double[]{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()});

        subLevel.updateMergedMassData(1.0f);
        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();

        if (centerOfMass != null) {
            subLevel.logicalPose().rotationPoint().set(centerOfMass);
            this.onStatsChanged(subLevel);
        }
    }

    /**
     * Removes a {@link SubLevel} from the physics pipeline.
     */
    @Override
    public void remove(final ServerSubLevel subLevel) {
        int id = Rapier3D.getID(subLevel);
        this.registeredSubLevels.remove(id);
        RapierPhysicsRegion region = this.getRegion(subLevel);
        if (region != null) {
            Rapier3D.removeSubLevel(region.getSceneHandle(), id);
            this.markRegionDirty(region);
        }
        if (this.universeHandle != 0) {
            Rapier3D.removeUniverseBody(this.universeHandle, id);
        }
        this.spatialIndex.removeSubLevel(subLevel);
    }

    /**
     * Adds a kinematic contraption to the scene
     */
    @Override
    public void add(final KinematicContraption contraption) {
        if (this.activeContraptions.containsKey(contraption)) {
            throw new IllegalStateException("Contraption " + contraption + " is already present in pipeline");
        }

        final int id = this.getNextRuntimeID();
        this.activeContraptions.put(contraption, new TrackedKinematicContraption(new Vector3d(), new Quaterniond(), new Vector3d(), new Vector3d(), id));

        final SubLevel mountSubLevel = Sable.HELPER.getContaining(this.level, contraption.sable$getPosition());
        final int mountId = mountSubLevel != null ? Rapier3D.getID((ServerSubLevel) mountSubLevel) : -1;

        final BoundingBox3i localBounds = new BoundingBox3i();
        contraption.sable$getLocalBounds(localBounds);

        final Vector3dc pos = contraption.sable$getPosition();
        final Quaterniond rot = contraption.sable$getOrientation();
        final double[] pose = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};

        Rapier3D.createKinematicContraption(this.getDefaultSceneHandle(), mountId, id, pose);

        // collect chunks
        record UploadingContraptionChunk(int[] data) {
        }
        final Long2ObjectMap<UploadingContraptionChunk> chunks = new Long2ObjectOpenHashMap<>();

        final BlockPos.MutableBlockPos blockPos = new BlockPos.MutableBlockPos();
        for (int x = localBounds.minX(); x <= localBounds.maxX(); x++) {
            for (int z = localBounds.minZ(); z <= localBounds.maxZ(); z++) {
                for (int y = localBounds.minY(); y <= localBounds.maxY(); y++) {
                    final BlockState blockState = contraption.sable$blockGetter().getBlockState(blockPos.set(x, y, z));

                    if (blockState.isAir()) continue;

                    final SectionPos sectionPos = SectionPos.of(blockPos);
                    final UploadingContraptionChunk chunk = chunks.computeIfAbsent(sectionPos.asLong(), longPos -> new UploadingContraptionChunk(new int[LevelChunkSection.SECTION_SIZE]));

                    final VoxelNeighborhoodState state = VoxelNeighborhoodState.CORNER;
                    final RapierVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(blockState);

                    final int index = (x & 15) + ((z & 15) << 4) + ((y & 15) << 8);

                    final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
                    chunk.data[index] = packBlockState(state, colliderValue);
                }
            }
        }

        if (contraption.sable$shouldCollide()) {
            for (final Long2ObjectMap.Entry<UploadingContraptionChunk> entry : chunks.long2ObjectEntrySet()) {
                final SectionPos sectionPos = SectionPos.of(entry.getLongKey());
                final UploadingContraptionChunk chunk = entry.getValue();
                Rapier3D.addKinematicContraptionChunkSection(this.getDefaultSceneHandle(), id, sectionPos.x(), sectionPos.y(), sectionPos.z(), chunk.data());
            }
        }

        this.updateContraptionPose(contraption, 1.0f);
        Rapier3D.setLocalBounds(this.getDefaultSceneHandle(), id, localBounds.minX, localBounds.minY, localBounds.minZ, localBounds.maxX, localBounds.maxY, localBounds.maxZ);
        this.markRegionDirty((RapierPhysicsRegion) this.spatialIndex.getDefaultRegion());
    }

    /**
     * Removes a kinematic contraption from the scene
     */
    @Override
    public void remove(final KinematicContraption contraption) {
        final TrackedKinematicContraption removed = this.activeContraptions.remove(contraption);

        if (removed == null) {
            return;
        }

        Rapier3D.removeKinematicContraption(this.getDefaultSceneHandle(), removed.id());
        this.markRegionDirty((RapierPhysicsRegion) this.spatialIndex.getDefaultRegion());
    }

    private ByteBuffer batchedPoseBuffer = null;

    @Override
    public void worldengine$syncActivePoses(ServerSubLevelContainer container, WorldEnginePhysicsSystem system) {
        if (this.spatialIndex == null) return;
        system.worldengine$beginPoseSync();

        // 1. Drain universe dirty poses (ballistic/nonresident/dirty bodies)
        if (this.universeHandle != 0) {
            int initialCapacityEntries = 256;
            int reqUniCap = initialCapacityEntries * 60;
            if (this.batchedPoseBuffer == null || this.batchedPoseBuffer.capacity() < reqUniCap) {
                this.batchedPoseBuffer = ByteBuffer.allocateDirect(reqUniCap).order(ByteOrder.nativeOrder());
            }
            int capacityEntries = this.batchedPoseBuffer.capacity() / 60;
            int writtenUniverse = Rapier3D.writeUniverseDirtyPoses(this.universeHandle, this.batchedPoseBuffer, capacityEntries);
            if (writtenUniverse < 0) {
                int requiredEntries = Math.negateExact(writtenUniverse);
                reqUniCap = Math.multiplyExact(requiredEntries, 60);
                this.batchedPoseBuffer = ByteBuffer.allocateDirect(reqUniCap).order(ByteOrder.nativeOrder());
                writtenUniverse = Rapier3D.writeUniverseDirtyPoses(this.universeHandle, this.batchedPoseBuffer, requiredEntries);
            }
            for (int i = 0; i < writtenUniverse; i++) {
                int offset = i * 60;
                int id = this.batchedPoseBuffer.getInt(offset);
                ServerSubLevel subLevel = this.registeredSubLevels.get(id);
                if (subLevel != null && !subLevel.isRemoved()) {
                    double px = this.batchedPoseBuffer.getDouble(offset + 4);
                    double py = this.batchedPoseBuffer.getDouble(offset + 12);
                    double pz = this.batchedPoseBuffer.getDouble(offset + 20);
                    double qx = this.batchedPoseBuffer.getDouble(offset + 28);
                    double qy = this.batchedPoseBuffer.getDouble(offset + 36);
                    double qz = this.batchedPoseBuffer.getDouble(offset + 44);
                    double qw = this.batchedPoseBuffer.getDouble(offset + 52);

                    system.worldengine$storagePose().position().set(px, py, pz);
                    system.worldengine$storagePose().orientation().set(qx, qy, qz, qw);
                    system.worldengine$applyStoragePose(subLevel);
                    this.spatialIndex.markBodyMoved(subLevel);
                }
            }
        }

        // 2. Sync active poses per stepped region
        for (RapierPhysicsRegion region : List.copyOf(this.steppedRegions)) {
            int maxBodies = Math.max(1, region.getActiveSubLevels().size() + this.activeContraptions.size());
            int requiredCapacity = maxBodies * 60;

            if (this.batchedPoseBuffer == null || this.batchedPoseBuffer.capacity() < requiredCapacity) {
                this.batchedPoseBuffer = ByteBuffer.allocateDirect(requiredCapacity).order(ByteOrder.nativeOrder());
            }

            int written = Rapier3D.writeActivePoses(region.getSceneHandle(), this.batchedPoseBuffer, maxBodies);
            if (written < 0) {
                maxBodies = Math.negateExact(written);
                requiredCapacity = Math.multiplyExact(maxBodies, 60);
                this.batchedPoseBuffer = ByteBuffer.allocateDirect(requiredCapacity).order(ByteOrder.nativeOrder());
                written = Rapier3D.writeActivePoses(region.getSceneHandle(), this.batchedPoseBuffer, maxBodies);
                if (written < 0) {
                    throw new IllegalStateException("Active pose count changed while resizing the native output buffer");
                }
            }

            for (int i = 0; i < written; i++) {
                int offset = i * 60;
                int id = this.batchedPoseBuffer.getInt(offset);
                
                ServerSubLevel subLevel = region.getSubLevel(id);
                if (subLevel != null && !subLevel.isRemoved()) {
                    system.worldengine$markActive(subLevel);
                    double px = this.batchedPoseBuffer.getDouble(offset + 4);
                    double py = this.batchedPoseBuffer.getDouble(offset + 12);
                    double pz = this.batchedPoseBuffer.getDouble(offset + 20);
                    double qx = this.batchedPoseBuffer.getDouble(offset + 28);
                    double qy = this.batchedPoseBuffer.getDouble(offset + 36);
                    double qz = this.batchedPoseBuffer.getDouble(offset + 44);
                    double qw = this.batchedPoseBuffer.getDouble(offset + 52);
                    
                    this.poseCache[0] = px;
                    this.poseCache[1] = py;
                    this.poseCache[2] = pz;
                    this.poseCache[3] = qx;
                    this.poseCache[4] = qy;
                    this.poseCache[5] = qz;
                    this.poseCache[6] = qw;
                    
                    // Set the storage pose for the system
                    system.worldengine$storagePose().position().set(px, py, pz);
                    system.worldengine$storagePose().orientation().set(qx, qy, qz, qw);
                    
                    system.worldengine$applyStoragePose(subLevel);
                    this.spatialIndex.markBodyMoved(subLevel);
                }
            }
            this.streamRegionTerrain(region);
            if (region.hasNativeActivity()) {
                region.invalidateSchedule();
                this.activeRegions.add(region);
            } else {
                this.activeRegions.remove(region);
                long delay = region.ticksUntilNextWake();
                if (delay != Long.MAX_VALUE) {
                    long generation = region.invalidateSchedule();
                    long wakeTick = this.physicsTickCounter + Math.max(1L, delay);
                    this.scheduledRegions.add(new ScheduledRegion(wakeTick, generation, region));
                }
            }
        }
        this.spatialIndex.tick();
        system.worldengine$endPoseSync();
    }

    void populateRegionTerrain(RapierPhysicsRegion region) {
        // Terrain is populated lazily by streamRegionTerrain once bodies have
        // been assigned. A newly-created empty region owns no world chunks.
    }

    private void addTerrainRange(LongSet desired, TerrainFootprintTracker.Envelope envelope) {
        if (envelope.isEmpty()) return;
        for (int x = envelope.minX(); x <= envelope.maxX(); x++) {
            for (int y = envelope.minY(); y <= envelope.maxY(); y++) {
                for (int z = envelope.minZ(); z <= envelope.maxZ(); z++) {
                    desired.add(SectionPos.asLong(x, y, z));
                }
            }
        }
    }

    private void addTerrainRange(LongSet desired, double minX, double minY, double minZ,
                                 double maxX, double maxY, double maxZ) {
        this.addTerrainRange(desired, TerrainFootprintTracker.Envelope.fromWorldBounds(
                minX, minY, minZ, maxX, maxY, maxZ));
    }

    private TerrainFootprintTracker.Envelope terrainEnvelope(ServerSubLevel subLevel) {
        var bounds = subLevel.boundingBox();
        Vector3dc velocity = subLevel.latestLinearVelocity;
        double dx = velocity.x() * 2.0;
        double dy = velocity.y() * 2.0;
        double dz = velocity.z() * 2.0;

        if (bounds.volume() > 0.0) {
            return TerrainFootprintTracker.Envelope.fromWorldBounds(
                    bounds.minX() + Math.min(0.0, dx) - 32.0,
                    bounds.minY() + Math.min(0.0, dy) - 32.0,
                    bounds.minZ() + Math.min(0.0, dz) - 32.0,
                    bounds.maxX() + Math.max(0.0, dx) + 32.0,
                    bounds.maxY() + Math.max(0.0, dy) + 32.0,
                    bounds.maxZ() + Math.max(0.0, dz) + 32.0);
        }

        // A sublevel is registered before assembly/load populates its plot, so
        // seed its initial footprint at the body pose until plot bounds exist.
        Vector3dc position = subLevel.logicalPose().position();
        return TerrainFootprintTracker.Envelope.fromWorldBounds(
                position.x() - 32.0, position.y() - 32.0, position.z() - 32.0,
                position.x() + 32.0, position.y() + 32.0, position.z() + 32.0);
    }

    void streamRegionTerrain(RapierPhysicsRegion region) {
        boolean changedNativeTerrain = false;
        for (int id : region.drainDirtyTerrainBodies()) {
            LongSet desired = new LongOpenHashSet();
            ServerSubLevel subLevel = region.getSubLevel(id);
            if (subLevel != null && !subLevel.isRemoved()) {
                TerrainFootprintTracker.Envelope envelope = this.terrainEnvelope(subLevel);
                // Pose updates mark the body as a cheap candidate every tick.
                // Only allocate and diff the section set after its conservative
                // swept section envelope actually changes.
                if (!region.terrainFootprintNeedsRefresh(id, envelope)) continue;
                this.addTerrainRange(desired, envelope);
            }
            region.replaceTerrainFootprint(id, desired);
        }

        // Non-sublevel objects are few and have no persistent body id. Keep
        // their footprint under one reserved owner while body footprints stay
        // fully incremental.
        if (region == this.spatialIndex.getDefaultRegion()) {
            LongSet desired = new LongOpenHashSet();
            for (KinematicContraption contraption : this.activeContraptions.keySet()) {
                Vector3dc pos = contraption.sable$getPosition();
                this.addTerrainRange(desired, pos.x() - 32.0, pos.y() - 32.0, pos.z() - 32.0,
                        pos.x() + 32.0, pos.y() + 32.0, pos.z() + 32.0);
            }
            BoundingBox3d objectBounds = new BoundingBox3d();
            this.activeBoxes.removeIf(box -> !box.isActive());
            for (BoxPhysicsObject box : this.activeBoxes) {
                box.getBoundingBox(objectBounds);
                this.addTerrainRange(desired, objectBounds.minX(), objectBounds.minY(), objectBounds.minZ(),
                        objectBounds.maxX(), objectBounds.maxY(), objectBounds.maxZ());
            }
            this.activeRopes.removeIf(rope -> !rope.isActive());
            for (RopePhysicsObject rope : this.activeRopes) {
                rope.getBoundingBox(objectBounds);
                this.addTerrainRange(desired, objectBounds.minX(), objectBounds.minY(), objectBounds.minZ(),
                        objectBounds.maxX(), objectBounds.maxY(), objectBounds.maxZ());
            }
            region.replaceTerrainFootprint(Integer.MIN_VALUE, desired);
        }

        for (long key : region.drainChangedTerrainSections()) {
            boolean required = region.getRequiredTerrainSections().contains(key);
            boolean loaded = region.getLoadedTerrainSections().contains(key);
            SectionPos section = SectionPos.of(key);
            if (!required && loaded) {
                Rapier3D.removeChunk(region.getSceneHandle(), section.x(), section.y(), section.z(), true, -1);
                region.getLoadedTerrainSections().remove(key);
                changedNativeTerrain = true;
            } else if (required && !loaded) {
                int[] data = this.globalChunkCache.get(key);
                if (data == null) continue;
                Rapier3D.addChunk(region.getSceneHandle(), section.x(), section.y(), section.z(), data, true, -1);
                region.getLoadedTerrainSections().add(key);
                changedNativeTerrain = true;
            }
        }
        if (changedNativeTerrain) this.markRegionDirty(region);
    }

    private static boolean isInsideMinecraftTerrainDomain(Vector3dc pos) {
        return pos.x() >= -30_000_000.0 && pos.x() <= 30_000_000.0
                && pos.z() >= -30_000_000.0 && pos.z() <= 30_000_000.0
                && pos.y() >= -2048.0 && pos.y() <= 2048.0;
    }

    /**
     * A body can be created before the normal physics chunk-ticket pass has
     * uploaded its surroundings. Seed a small collision neighborhood so its
     * first simulation step cannot run against an empty region.
     */
    void ensureTerrainNear(Vector3dc position) {
        if (!isInsideMinecraftTerrainDomain(position)) return;
        int centerX = ((int) Math.floor(position.x())) >> 4;
        int centerY = ((int) Math.floor(position.y())) >> 4;
        int centerZ = ((int) Math.floor(position.z())) >> 4;
        for (int x = centerX - 1; x <= centerX + 1; x++) {
            for (int z = centerZ - 1; z <= centerZ + 1; z++) {
                LevelChunk chunk = this.accelerator.getChunk(x, z);
                for (int y = centerY - 1; y <= centerY + 1; y++) {
                    if (y < this.level.getMinSection() || y >= this.level.getMaxSection()) continue;
                    long key = SectionPos.asLong(x, y, z);
                    if (this.globalChunkCache.containsKey(key)) continue;
                    this.handleChunkSectionAddition(
                            chunk.getSection(this.level.getSectionIndexFromSectionY(y)),
                            x, y, z, false);
                }
            }
        }
    }

    private void updateCachedWorldBlock(int x, int y, int z, int packedState) {
        int[] section = this.globalChunkCache.get(SectionPos.asLong(x >> 4, y >> 4, z >> 4));
        if (section != null) {
            int index = (x & 15) + ((z & 15) << 4) + ((y & 15) << 8);
            section[index] = packedState;
        }
    }

    /**
     * Queries the physics pipeline for the current pose of a {@link SubLevel}.
     */
    @Override
    public Pose3d readPose(final ServerSubLevel subLevel, final Pose3d dest) {
        this.assertBodyValid(subLevel);
        if (this.universeHandle != 0) {
            Rapier3D.getUniversePose(this.universeHandle, Rapier3D.getID(subLevel), this.poseCache);
            dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
            dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);
            return dest;
        }
        long handle = getSceneHandle(subLevel);
        if (handle == 0) return dest;
        Rapier3D.getPose(handle, Rapier3D.getID(subLevel), this.poseCache);

        dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);

        return dest;
    }

    /**
     * Adds a rope to the physics pipeline
     */
    @Override
    public RopeHandle addRope(final RopePhysicsObject rope) {
        this.activeRopes.add(rope);
        if (!rope.getPoints().isEmpty()) this.ensureTerrainNear(rope.getPoints().getFirst());
        RopeHandle handle = RapierRopeHandle.create(this.getDefaultSceneHandle(), rope.getCollisionRadius(), rope.getPoints());
        RapierPhysicsRegion region = (RapierPhysicsRegion) this.spatialIndex.getDefaultRegion();
        this.streamRegionTerrain(region);
        this.markRegionDirty(region);
        return handle;
    }

    /**
     * Adds a box to the physics pipeline
     */
    @Override
    public BoxHandle addBox(final BoxPhysicsObject box) {
        this.activeBoxes.add(box);
        this.ensureTerrainNear(box.getPose().position());
        BoxHandle handle = RapierBoxHandle.create(this.getDefaultSceneHandle(), box.getPose(), box.getHalfExtents(), box.getMass());
        RapierPhysicsRegion region = (RapierPhysicsRegion) this.spatialIndex.getDefaultRegion();
        this.streamRegionTerrain(region);
        this.markRegionDirty(region);
        return handle;
    }

    /**
     * Handles the addition of a chunk section to the physics context
     */
    @Override
    public void handleChunkSectionAddition(final LevelChunkSection section, final int x, final int y, final int z, final boolean uploadDataIfGlobal) {
        this.accelerator.clearCache();

        // this means the x coordinate is the fastest changing, then z, then y
        final int[] array = new int[LevelChunkSection.SECTION_SIZE];

        final SectionPos sectionPos = SectionPos.of(x, y, z);

        // if it's only air, all zeros will do. it'll default to empty neighborhood state and 0 (empty) collider ID
        if (!section.hasOnlyAir()) {
            final LevelChunk chunk = this.accelerator.getChunk(x, z);

            for (int bx = 0; bx < 16; bx++) {
                for (int bz = 0; bz < 16; bz++) {
                    for (int by = 0; by < 16; by++) {
                        final BlockPos globalPos = new BlockPos(bx, by, bz).offset(sectionPos.minBlockX(), sectionPos.minBlockY(), sectionPos.minBlockZ());
                        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, globalPos, chunk);
                        final RapierVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(this.accelerator.getBlockState(globalPos));

                        final int index = bx + (bz << 4) + (by << 8);

                        final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
                        array[index] = packBlockState(state, colliderValue);
                    }
                }
            }
        }

        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(x, z);
        final boolean global = plot == null;
        int id = -1;

        if (plot != null && uploadDataIfGlobal) id = Rapier3D.getID(((ServerSubLevel) plot.getSubLevel()));
        if (this.spatialIndex != null) {
            if (global) {
                this.globalChunkCache.put(sectionPos.asLong(), array);
                if (this.universeHandle != 0) {
                    Rapier3D.addWorldTerrainChunk(this.universeHandle, x, y, z);
                }
                for (RapierPhysicsRegion region : this.terrainRegions(sectionPos.asLong())) {
                    Rapier3D.addChunk(region.getSceneHandle(), x, y, z, array, true, -1);
                    region.getLoadedTerrainSections().add(sectionPos.asLong());
                    this.markRegionDirty(region);
                }
            } else if (id != -1) {
                if (this.universeHandle != 0) {
                    Rapier3D.addUniverseSubLevelChunk(this.universeHandle, id, x, y, z, array);
                }
                RapierPhysicsRegion region = this.spatialIndex.getRegion(id);
                if (region != null) {
                    Rapier3D.addChunk(region.getSceneHandle(), x, y, z, array, false, id);
                    this.markRegionDirty(region);
                }
            }
        }
    }

    /**
     * Handles the removal of a chunk section from the physics context
     */
    @Override
    public void handleChunkSectionRemoval(final int x, final int y, final int z) {
        if (this.spatialIndex == null) return;
        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(x, z);
        if (plot == null) {
            long section = SectionPos.asLong(x, y, z);
            this.globalChunkCache.remove(section);
            if (this.universeHandle != 0) {
                Rapier3D.removeWorldTerrainChunk(this.universeHandle, x, y, z);
            }
            for (RapierPhysicsRegion region : this.terrainRegions(section)) {
                if (region.getLoadedTerrainSections().remove(section)) {
                    Rapier3D.removeChunk(region.getSceneHandle(), x, y, z, true, -1);
                    this.markRegionDirty(region);
                }
            }
        } else {
            int id = Rapier3D.getID((ServerSubLevel) plot.getSubLevel());
            if (this.universeHandle != 0) {
                Rapier3D.removeUniverseSubLevelChunk(this.universeHandle, id, x, y, z);
            }
            RapierPhysicsRegion region = this.spatialIndex.getRegion(id);
            if (region != null) {
                Rapier3D.removeChunk(region.getSceneHandle(), x, y, z, false, id);
                this.markRegionDirty(region);
            }
        }
    }

    /**
     * Handles the change of a block (from oldState to newState) in a chunk at chunk-relative position x, y, z.
     * Only called server-side.
     *
     * @param x chunk-relative x position
     * @param y chunk-relative y position
     * @param z chunk-relative z position
     */
    @Override
    public void handleBlockChange(final SectionPos sectionPos, final LevelChunkSection chunk, int x, int y, int z, final BlockState oldState, final BlockState newState) {
        x = (sectionPos.x() << 4) + x;
        y = (sectionPos.y() << 4) + y;
        z = (sectionPos.z() << 4) + z;

        final BlockPos globalBlockPos = new BlockPos(x, y, z);
        
        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(x >> 4, z >> 4);
        final boolean isSubLevel = plot != null;
        final int subLevelId = isSubLevel ? Rapier3D.getID((ServerSubLevel) plot.getSubLevel()) : -1;

        for (final Direction dir : Direction.values()) {
            final BlockPos pos = globalBlockPos.relative(dir);
            final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, pos, null);
            final RapierVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(this.level.getBlockState(pos));

            final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
            final int packedState = packBlockState(state, colliderValue);
            
            final LevelPlot offsetPlot = SubLevelContainer.getContainer(this.level).getPlot(pos.getX() >> 4, pos.getZ() >> 4);
            final boolean offsetIsSubLevel = offsetPlot != null;
            if (this.spatialIndex != null) {
                if (offsetIsSubLevel) {
                    int bodyId = Rapier3D.getID((ServerSubLevel) offsetPlot.getSubLevel());
                    if (this.universeHandle != 0) {
                        Rapier3D.changeUniverseSubLevelBlock(this.universeHandle, bodyId, pos.getX(), pos.getY(), pos.getZ(), packedState);
                    }
                    RapierPhysicsRegion region = this.spatialIndex.getRegion(bodyId);
                    if (region != null) {
                        this.markRegionDirty(region);
                    }
                } else {
                    this.updateCachedWorldBlock(pos.getX(), pos.getY(), pos.getZ(), packedState);
                    long neighborSection = SectionPos.asLong(pos.getX() >> 4, pos.getY() >> 4, pos.getZ() >> 4);
                    for (RapierPhysicsRegion region : this.terrainRegions(neighborSection)) {
                        if (region.getLoadedTerrainSections().contains(neighborSection)) {
                            Rapier3D.changeWorldBlock(region.getSceneHandle(), pos.getX(), pos.getY(), pos.getZ(), packedState);
                            this.markRegionDirty(region);
                        }
                    }
                }
            }
        }

        // do it for the block without offset
        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, globalBlockPos, null);
        final RapierVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(newState);

        final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
        final int packedState = packBlockState(state, colliderValue);
        if (this.spatialIndex != null) {
            if (isSubLevel) {
                if (this.universeHandle != 0) {
                    Rapier3D.changeUniverseSubLevelBlock(this.universeHandle, subLevelId, x, y, z, packedState);
                }
                RapierPhysicsRegion region = this.spatialIndex.getRegion(subLevelId);
                if (region != null) {
                    this.markRegionDirty(region);
                }
            } else {
                this.updateCachedWorldBlock(x, y, z, packedState);
                for (RapierPhysicsRegion region : this.terrainRegions(sectionPos.asLong())) {
                    if (region.getLoadedTerrainSections().contains(sectionPos.asLong())) {
                        Rapier3D.changeWorldBlock(region.getSceneHandle(), x, y, z, packedState);
                        this.markRegionDirty(region);
                    }
                }
            }
        }
    }

    @Override
    public void onStatsChanged(@NotNull final ServerSubLevel subLevel) {
        this.assertBodyValid(subLevel);

        final BoundingBox3ic plotBounds = subLevel.getPlot().getBoundingBox();
        final int id = Rapier3D.getID(subLevel);

        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();
        final org.joml.Matrix3dc inertiaTensor = subLevel.getMassTracker().getInertiaTensor();
        final double mass = subLevel.getMassTracker().getMass();
        final double[] centerOfMassArray = centerOfMass != null ? new double[]{centerOfMass.x(), centerOfMass.y(), centerOfMass.z()} : null;
        final double[] inertiaTensorArray = inertiaTensor != null ? new double[]{
                inertiaTensor.m00(), inertiaTensor.m01(), inertiaTensor.m02(),
                inertiaTensor.m10(), inertiaTensor.m11(), inertiaTensor.m12(),
                inertiaTensor.m20(), inertiaTensor.m21(), inertiaTensor.m22()
        } : null;

        if (this.universeHandle != 0) {
            Rapier3D.setUniverseBodyStats(this.universeHandle, id, mass, centerOfMassArray, inertiaTensorArray,
                    plotBounds.minX(), plotBounds.minY(), plotBounds.minZ(),
                    plotBounds.maxX(), plotBounds.maxY(), plotBounds.maxZ());
        }

        RapierPhysicsRegion region = this.getRegion(subLevel);
        if (region != null) {
            this.spatialIndex.markBodyMoved(subLevel);
            this.markRegionDirty(region);
        }
    }

    /**
     * Teleports the physics body of a sub-level to a given position.
     *
     * @param body    the physics pipeline body to teleport
     * @param position    the new position to teleport to
     * @param orientation the new orientation to teleport to
     */
    @Override
    public void teleport(final PhysicsPipelineBody body, final Vector3dc position, final Quaterniondc orientation) {
        this.assertBodyValid(body);
        Rapier3D.teleportUniverse(this.universeHandle, Rapier3D.getID(body), position.x(), position.y(), position.z(), orientation.x(), orientation.y(), orientation.z(), orientation.w());
        RapierPhysicsRegion region = this.getRegion(body);
        if (region != null) this.markRegionDirty(region);
        if (body instanceof final ServerSubLevel subLevel) {
            subLevel.logicalPose().position().set(position);
            subLevel.logicalPose().orientation().set(orientation);
        }
    }

    /**
     * Adds a force at a given world position to a sub-level containing the position
     *
     * @param body the sub-level to apply the force to
     * @param position the position to apply the force at [m]
     * @param force    the force to apply [N]
     */
    @Override
    public void applyImpulse(final PhysicsPipelineBody body, final Vector3dc position, final Vector3dc force) {
        this.assertBodyValid(body);
        final Vector3dc centerOfMass = body.getMassTracker().getCenterOfMass();
        double relX = centerOfMass == null ? position.x() : position.x() - centerOfMass.x();
        double relY = centerOfMass == null ? position.y() : position.y() - centerOfMass.y();
        double relZ = centerOfMass == null ? position.z() : position.z() - centerOfMass.z();
        Rapier3D.applyImpulseUniverse(this.universeHandle, Rapier3D.getID(body), relX, relY, relZ, force.x(), force.y(), force.z());
        RapierPhysicsRegion region = this.getRegion(body);
        if (region != null) this.markRegionDirty(region);
    }

    /**
     * Adds a local force and torque
     *
     * @param body the sub-level to apply the force to
     * @param torque   the local torque to apply [Nm]
     */
    @Override
    public void applyLinearAndAngularImpulse(final PhysicsPipelineBody body, final Vector3dc force, final Vector3dc torque, final boolean wakeUp) {
        this.assertBodyValid(body);
        Rapier3D.applyForceAndTorqueUniverse(this.universeHandle, Rapier3D.getID(body), force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z(), wakeUp);
        RapierPhysicsRegion region = this.getRegion(body);
        if (region != null) this.markRegionDirty(region);
    }

    /**
     * Adds linear and angular velocities to a sub-level
     *
     * @param body        the sub-level to apply the velocities to
     * @param linearVelocity  the linear velocity to apply [m/s]
     * @param angularVelocity the angular velocity to apply [rad/s]
     */
    @Override
    public void addLinearAndAngularVelocity(final PhysicsPipelineBody body, final Vector3dc linearVelocity, final Vector3dc angularVelocity) {
        this.assertBodyValid(body);
        Rapier3D.addLinearAngularVelocityUniverse(this.universeHandle, Rapier3D.getID(body), linearVelocity.x(), linearVelocity.y(), linearVelocity.z(), angularVelocity.x(), angularVelocity.y(), angularVelocity.z(), true);
        RapierPhysicsRegion region = this.getRegion(body);
        if (region != null) this.markRegionDirty(region);
    }

    @Override
    public Vector3d getLinearVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        if (this.universeHandle != 0) {
            Rapier3D.getUniverseLinearVelocity(this.universeHandle, Rapier3D.getID(body), this.poseCache);
            return dest.set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        }
        long sceneHandle = getSceneHandle(body);
        if (sceneHandle != 0) {
            Rapier3D.getLinearVelocity(sceneHandle, Rapier3D.getID(body), this.poseCache);
            return dest.set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        }
        return dest.zero();
    }

    @Override
    public Vector3d getAngularVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        if (this.universeHandle != 0) {
            Rapier3D.getUniverseAngularVelocity(this.universeHandle, Rapier3D.getID(body), this.poseCache);
            return dest.set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        }
        long sceneHandle = getSceneHandle(body);
        if (sceneHandle != 0) {
            Rapier3D.getAngularVelocity(sceneHandle, Rapier3D.getID(body), this.poseCache);
            return dest.set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        }
        return dest.zero();
    }

    /**
     * "Wakes up" a sub-level, indicating environmental or other changes have occurred that should resume physics for idled or sleeping sub-levels.
     *
     * @param body the sub-level to wake up
     */
    @Override
    public void wakeUp(final PhysicsPipelineBody body) {
        this.assertBodyValid(body);
        if (body instanceof ServerSubLevel subLevel) {
            SubLevelPhysicsSystem system = SubLevelPhysicsSystem.get(this.level);
            if (system instanceof WorldEnginePhysicsSystem optimized) optimized.worldengine$activate(subLevel);
        }

        Rapier3D.wakeUpUniverse(this.universeHandle, Rapier3D.getID(body));
        RapierPhysicsRegion region = this.getRegion(body);
        if (region != null) this.markRegionDirty(region);
    }

    /**
     * Adds a constraint to the engine, returning its handle
     *
     * @param bodyA     the first sub-level to constrain, or null to constrain the second sub-level to the world
     * @param bodyB     the second sub-level to constrain, or null to constrain the first sub-level to the world
     * @param configuration the configuration of the constraint
     */
    @SuppressWarnings("unchecked")
    @Override
    @Nullable
    public <T extends PhysicsConstraintHandle> T addConstraint(@Nullable final PhysicsPipelineBody bodyA, @Nullable final PhysicsPipelineBody bodyB, @NotNull final PhysicsConstraintConfiguration<T> configuration) {
        if (bodyA == null && bodyB == null) {
            throw new IllegalArgumentException("Cannot add a constraint between the static world and static world");
        }

        if (bodyA == bodyB) {
            throw new IllegalArgumentException("Cannot add a constraint between a body and itself");
        }

        try {
            configuration.validate(ServerSubLevelContainer.getContainer(this.level), bodyA, bodyB);
        } catch (final Exception e) {
            throw new IllegalArgumentException("Constraint validation failed", e);
        }

        final T constraint = switch (configuration) {
            case final RotaryConstraintConfiguration config ->
                    (T) RapierRotaryConstraintHandle.create(this.level, bodyA, bodyB, config);
            case final FixedConstraintConfiguration config ->
                    (T) RapierFixedConstraintHandle.create(this.level, bodyA, bodyB, config);
            case final FreeConstraintConfiguration config ->
                    (T) RapierFreeConstraintHandle.create(this.level, bodyA, bodyB, config);
            case final GenericConstraintConfiguration config ->
                    (T) RapierGenericConstraintHandle.create(this.level, bodyA, bodyB, config);
        };

        if (!constraint.isValid()) {
            return null;
        }

        this.markRegionDirty(bodyA != null ? this.getRegion(bodyA) : this.getRegion(bodyB));

        return constraint;
    }

    /**
     * Updates the config of the physics engine from a data object
     *
     * @param data the data to update from
     */
    @Override
    public void updateConfigFrom(final PhysicsConfigData data) {
        if (this.spatialIndex == null) return;
        for (PhysicsRegion region : this.spatialIndex.getRegions()) {
            long sceneHandle = region.getSceneHandle();
            Rapier3D.configFrequencyAndDamping(sceneHandle, data.contactSpringFrequency, data.contactSpringDampingRatio);
            Rapier3D.configSolverIterations(sceneHandle, data.solverIterations, data.pgsIterations, data.stabilizationIterations);
            Rapier3D.configMinIslandSize(sceneHandle, data.minDynamicBodiesPerIsland);
        }
    }

    /**
     * @return the next runtime ID for a collider / sub-level
     */
    @Override
    public int getNextRuntimeID() {
        return Rapier3D.nextBodyID();
    }

    private void assertBodyValid(final PhysicsPipelineBody body) {
        if (body.isRemoved()) {
            throw new RuntimeException("Body has been removed");
        }
    }

    private void updateContraptionPoses() {
        final SubLevelPhysicsSystem system = SubLevelPhysicsSystem.require(this.level);
        final double partialPhysicsTick = system.getPartialPhysicsTick();

        for (final KinematicContraption contraption : this.activeContraptions.keySet()) {
            this.updateContraptionPose(contraption, partialPhysicsTick);
        }
    }

    private void updateContraptionPose(final KinematicContraption contraption, final double partialPhysicsTick) {
        final TrackedKinematicContraption trackedContraption = this.activeContraptions.get(contraption);

        final SubLevel mountSubLevel = Sable.HELPER.getContaining(this.level, contraption.sable$getPosition());
        final Vector3dc parentCenterOfMass = mountSubLevel != null ? ((ServerSubLevel) mountSubLevel).getMassTracker().getCenterOfMass() : JOMLConversion.ZERO;

        final Vector3dc lastPosition = new Vector3d(contraption.sable$getPosition(partialPhysicsTick - 1.0f));
        final Quaterniondc lastOrientation = new Quaterniond(contraption.sable$getOrientation(partialPhysicsTick - 1.0f));

        final Vector3d pos = new Vector3d(contraption.sable$getPosition(partialPhysicsTick));
        final Quaterniondc rot = contraption.sable$getOrientation(partialPhysicsTick);

        final Vector3d linVel = pos.sub(lastPosition, new Vector3d());
        final Vector3d angVel = SableMathUtils.getAngularVelocity(lastOrientation, rot, new Vector3d());

        linVel.mul(20.0);
        angVel.mul(20.0);
        rot.transformInverse(linVel);
        rot.transformInverse(angVel);

        pos.sub(parentCenterOfMass);

        if (
                pos.distanceSquared(trackedContraption.lastUploadedPosition()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        linVel.distanceSquared(trackedContraption.lastUploadedLinVel()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        angVel.distanceSquared(trackedContraption.lastUploadedAngVel()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        rot.div(trackedContraption.lastUploadedOrientation(), new Quaterniond()).angle() > ANGULAR_THRESHOLD * ANGULAR_THRESHOLD
        ) {
            final MassTracker massTracker = contraption.sable$getMassTracker();
            final Vector3dc centerOfMass = massTracker.getCenterOfMass();

            final double[] centerOfMassArray = new double[]{centerOfMass.x(), centerOfMass.y(), centerOfMass.z()};
            final double[] poseArray = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};
            final double[] velocityArray = {linVel.x(), linVel.y(), linVel.z(), angVel.x(), angVel.y(), angVel.z()};
            Rapier3D.setKinematicContraptionTransform(this.universeHandle, trackedContraption.id(), centerOfMassArray, poseArray, velocityArray);
            this.markRegionDirty((RapierPhysicsRegion) this.spatialIndex.getDefaultRegion());

            trackedContraption.lastUploadedPosition().set(pos);
            trackedContraption.lastUploadedLinVel().set(linVel);
            trackedContraption.lastUploadedAngVel().set(angVel);
            trackedContraption.lastUploadedOrientation().set(rot);
        }
    }

    private void processCollisionEffects() {
        this.recentCollisions.long2LongEntrySet().removeIf(entry -> this.level.getGameTime() - entry.getLongValue() > 2);

        final Vector3d localPointA = new Vector3d();
        final Vector3d localPointB = new Vector3d();
        final Vector3d localNormalA = new Vector3d();
        final Vector3d localNormalB = new Vector3d();

        final Vector3d globalPointA = new Vector3d();
        final Vector3d globalPointB = new Vector3d();

        if (this.spatialIndex == null) return;
        for (RapierPhysicsRegion region : List.copyOf(this.steppedRegions)) {
            final double[] collisions = Rapier3D.clearCollisions(region.getSceneHandle());

            final BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
            final BlockPos.MutableBlockPos cornerPos = new BlockPos.MutableBlockPos();

            for (int i = 0; i < collisions.length / 15; i++) {
                final int startIndex = i * 15;
                final int idA = (int) collisions[startIndex];
                final int idB = (int) collisions[startIndex + 1];

                final double forceAmount = collisions[startIndex + 2];
                localNormalA.set(collisions[startIndex + 3], collisions[startIndex + 4], collisions[startIndex + 5]);
                localNormalB.set(collisions[startIndex + 6], collisions[startIndex + 7], collisions[startIndex + 8]);
                localPointA.set(collisions[startIndex + 9], collisions[startIndex + 10], collisions[startIndex + 11]);
                localPointB.set(collisions[startIndex + 12], collisions[startIndex + 13], collisions[startIndex + 14]);

                final ServerSubLevel subLevelA = region.getSubLevel(idA);
                final ServerSubLevel subLevelB = region.getSubLevel(idB);

            final double minMass = Math.min(subLevelA != null ? subLevelA.getMassTracker().getMass() : Double.MAX_VALUE, subLevelB != null ? subLevelB.getMassTracker().getMass() : Double.MAX_VALUE);

            if (forceAmount > 25.0 * minMass) {
                BlockState stateA = Blocks.STONE.defaultBlockState();
                BlockState stateB = stateA;

                if (subLevelA != null) {
                    final Pose3d pose = subLevelA.logicalPose();
                    pos.set(localPointA.x + pose.rotationPoint().x, localPointA.y + pose.rotationPoint().y, localPointA.z + pose.rotationPoint().z);
                    cornerPos.set(localPointA.x + pose.rotationPoint().x + 0.5, localPointA.y + pose.rotationPoint().y + 0.5, localPointA.z + pose.rotationPoint().z + 0.5);

                    final long exists = this.recentCollisions.put(cornerPos.asLong(), this.level.getGameTime());

                    if (exists != -1) {
                        continue;
                    }

                    stateA = this.accelerator.getBlockState(pos);
                }

                if (subLevelB != null) {
                    final Pose3d pose = subLevelB.logicalPose();
                    pos.set(localPointB.x + pose.rotationPoint().x, localPointB.y + pose.rotationPoint().y, localPointB.z + pose.rotationPoint().z);
                    cornerPos.set(localPointB.x + pose.rotationPoint().x + 0.5, localPointB.y + pose.rotationPoint().y + 0.5, localPointB.z + pose.rotationPoint().z + 0.5);

                    final long exists = this.recentCollisions.put(cornerPos.asLong(), this.level.getGameTime());

                    if (exists != -1) {
                        continue;
                    }

                    stateB = this.accelerator.getBlockState(pos);
                }

                globalPointA.set(localPointA);
                globalPointB.set(localPointB);

                if (subLevelA != null) {
                    final Pose3d pose = subLevelA.logicalPose();
                    pose.orientation().transform(globalPointA).add(pose.position());
                }

                if (subLevelB != null) {
                    final Pose3d pose = subLevelB.logicalPose();
                    pose.orientation().transform(globalPointB).add(pose.position());
                }

                final BlockState state = stateB;
                this.level.sendParticles(new BlockParticleOption(ParticleTypes.BLOCK, state), globalPointA.x, globalPointA.y, globalPointA.z, 2, 0.0, 0.0, 0.0, 0.1);

                final Vec3 position = JOMLConversion.toMojang(globalPointA);
                final float volumeScale = 0.4f;
                final SoundType soundType = state.getSoundType();

                this.level.playSound(null, position.x, position.y, position.z, soundType.getStepSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (0.6 - 0.2 + Math.random() * 0.4));
                this.level.playSound(null, position.x, position.y, position.z, soundType.getHitSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (Math.random() * 0.4));
                this.level.playSound(null, position.x, position.y, position.z, soundType.getPlaceSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (0.5 - 0.2 + Math.random() * 0.4));
            }
        }
        }
    }

    private record TrackedKinematicContraption(Vector3d lastUploadedPosition, Quaterniond lastUploadedOrientation,
                                               Vector3d lastUploadedLinVel, Vector3d lastUploadedAngVel, int id) {
    }

}
