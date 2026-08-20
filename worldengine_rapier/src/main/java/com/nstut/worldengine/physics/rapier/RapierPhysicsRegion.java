package com.nstut.worldengine.physics.rapier;

import dev.ryanhcode.sable.Sable;
import com.nstut.worldengine.api.PhysicsRegion;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.longs.LongSet;
import net.minecraft.CrashReport;
import net.minecraft.CrashReportCategory;
import net.minecraft.ReportedException;
import org.joml.Vector3dc;
import org.joml.Vector3d;

import java.util.Collection;

public class RapierPhysicsRegion implements PhysicsRegion {
    private final RapierPhysicsScene scene;
    private final RapierPhysicsPipeline owner;
    private final Vector3d origin;
    private final Int2ObjectMap<ServerSubLevel> activeSubLevels = new Int2ObjectOpenHashMap<>();
    private final LongSet requiredTerrainSections = new LongOpenHashSet();
    private final LongSet loadedTerrainSections = new LongOpenHashSet();
    private final Int2ObjectMap<LongSet> terrainFootprints = new Int2ObjectOpenHashMap<>();
    private final Long2IntOpenHashMap terrainReferenceCounts = new Long2IntOpenHashMap();
    private final TerrainFootprintTracker terrainFootprintTracker = new TerrainFootprintTracker();
    private final LongSet changedTerrainSections = new LongOpenHashSet();
    private long scheduleGeneration;
    private long lastStepTick;

    public RapierPhysicsRegion(RapierPhysicsPipeline owner, Vector3dc gravity, double universalDrag, Vector3dc origin) {
        this.owner = owner;
        this.origin = new Vector3d(origin);
        try {
            this.scene = new RapierPhysicsScene(Rapier3D.initialize(
                    owner.getUniverseHandle(),
                    gravity.x(), gravity.y(), gravity.z(), universalDrag,
                    origin.x(), origin.y(), origin.z()));
        } catch (final UnsatisfiedLinkError e) {
            Sable.LOGGER.error("Sable has failed to link with the natives for its Rapier pipeline. Please report with system details to " + Sable.ISSUE_TRACKER_URL, e);
            final CrashReport crashReport = CrashReport.forThrowable(e.getCause(), "Sable linking with Rapier natives");
            final CrashReportCategory category = crashReport.addCategory("Natives");
            category.setDetail("Name", Rapier3D.NATIVE_NAME);
            throw new ReportedException(crashReport);
        }
    }

    public Vector3dc getOrigin() {
        return this.origin;
    }

    public void rebaseOrigin(Vector3dc newOrigin) {
        Rapier3D.rebaseRegionOrigin(this.getSceneHandle(), newOrigin.x(), newOrigin.y(), newOrigin.z());
        this.origin.set(newOrigin);
        this.terrainFootprints.clear();
        this.terrainReferenceCounts.clear();
        this.loadedTerrainSections.clear();
        this.requiredTerrainSections.clear();
        this.changedTerrainSections.clear();
        this.terrainFootprintTracker.reset(this.activeSubLevels.keySet().toIntArray());
    }

    @Override
    public long getSceneHandle() {
        return this.scene.handle();
    }

    @Override
    public Collection<ServerSubLevel> getActiveSubLevels() {
        return this.activeSubLevels.values();
    }

    public ServerSubLevel getSubLevel(int id) {
        return this.activeSubLevels.get(id);
    }

    LongSet getRequiredTerrainSections() {
        return this.requiredTerrainSections;
    }

    LongSet getLoadedTerrainSections() {
        return this.loadedTerrainSections;
    }

    void markTerrainDirty(int id) {
        this.terrainFootprintTracker.markDirty(id);
    }

    void forceTerrainDirty(int id) {
        this.terrainFootprintTracker.forceDirty(id);
    }

    boolean terrainFootprintNeedsRefresh(int id, TerrainFootprintTracker.Envelope envelope) {
        return this.terrainFootprintTracker.needsRefresh(id, envelope);
    }

    int[] drainDirtyTerrainBodies() {
        return this.terrainFootprintTracker.drainDirtyBodies();
    }

    void replaceTerrainFootprint(int id, LongSet replacement) {
        LongSet previous = this.terrainFootprints.remove(id);
        if (previous != null) {
            for (long key : previous) {
                if (replacement.contains(key)) continue;
                int references = this.terrainReferenceCounts.addTo(key, -1) - 1;
                if (references <= 0) {
                    this.terrainReferenceCounts.remove(key);
                    this.requiredTerrainSections.remove(key);
                    this.changedTerrainSections.add(key);
                    this.owner.removeTerrainInterest(this, key);
                }
            }
        }
        for (long key : replacement) {
            if (previous != null && previous.contains(key)) continue;
            int references = this.terrainReferenceCounts.addTo(key, 1);
            if (references == 0) {
                this.requiredTerrainSections.add(key);
                this.changedTerrainSections.add(key);
                this.owner.addTerrainInterest(this, key);
            }
        }
        if (!replacement.isEmpty()) this.terrainFootprints.put(id, replacement);
    }

    LongSet drainChangedTerrainSections() {
        LongSet result = new LongOpenHashSet(this.changedTerrainSections);
        this.changedTerrainSections.clear();
        return result;
    }

    @Override
    public void addSubLevel(ServerSubLevel subLevel) {
        int id = Rapier3D.getID(subLevel);
        this.activeSubLevels.put(id, subLevel);
        this.forceTerrainDirty(id);
    }

    @Override
    public void removeSubLevel(ServerSubLevel subLevel) {
        int id = Rapier3D.getID(subLevel);
        this.activeSubLevels.remove(id);
        this.terrainFootprintTracker.remove(id);
        this.replaceTerrainFootprint(id, new LongOpenHashSet());
    }
    
    private static final int INITIAL_COMMAND_BUFFER_CAPACITY = 4096;
    private static final int COMMAND_MAGIC = 0x5341424C;
    private static final short COMMAND_PROTOCOL_VERSION = 1;
    private static final int COMMAND_HEADER_SIZE = 14;
    private java.nio.ByteBuffer commandBuffer = java.nio.ByteBuffer.allocateDirect(INITIAL_COMMAND_BUFFER_CAPACITY).order(java.nio.ByteOrder.nativeOrder());
    private int commandCount;

    private java.nio.ByteBuffer beginCommand(int requiredBytes) {
        if (requiredBytes < 0) {
            throw new IllegalArgumentException("requiredBytes must be non-negative");
        }
        int headerBytes = this.commandCount == 0 ? COMMAND_HEADER_SIZE : 0;
        int totalRequiredBytes = Math.addExact(headerBytes, requiredBytes);
        if (this.commandBuffer.remaining() < totalRequiredBytes) {
            int requiredCapacity = Math.addExact(this.commandBuffer.position(), totalRequiredBytes);
            int newCapacity = this.commandBuffer.capacity();
            while (newCapacity < requiredCapacity) {
                newCapacity = Math.multiplyExact(newCapacity, 2);
            }
            java.nio.ByteBuffer replacement = java.nio.ByteBuffer.allocateDirect(newCapacity).order(java.nio.ByteOrder.nativeOrder());
            this.commandBuffer.flip();
            replacement.put(this.commandBuffer);
            this.commandBuffer = replacement;
        }
        if (this.commandCount == 0) {
            this.commandBuffer.putInt(COMMAND_MAGIC);
            this.commandBuffer.putShort(COMMAND_PROTOCOL_VERSION);
            this.commandBuffer.putInt(0);
            this.commandBuffer.putInt(0);
        }
        this.commandCount = Math.incrementExact(this.commandCount);
        this.owner.markRegionDirty(this);
        return this.commandBuffer;
    }

    public void queueApplyImpulse(int id, double px, double py, double pz, double fx, double fy, double fz, boolean wakeUp) {
        java.nio.ByteBuffer buffer = this.beginCommand(54);
        buffer.put((byte) 1).putInt(id);
        buffer.putDouble(px).putDouble(py).putDouble(pz);
        buffer.putDouble(fx).putDouble(fy).putDouble(fz);
        buffer.put((byte) (wakeUp ? 1 : 0));
    }

    public void queueLinearAndAngularImpulse(int id, double fx, double fy, double fz, double tx, double ty, double tz, boolean wakeUp) {
        java.nio.ByteBuffer buffer = this.beginCommand(54);
        buffer.put((byte) 2).putInt(id);
        buffer.putDouble(fx).putDouble(fy).putDouble(fz);
        buffer.putDouble(tx).putDouble(ty).putDouble(tz);
        buffer.put((byte) (wakeUp ? 1 : 0));
    }

    public void queueLinearAndAngularVelocity(int id, double vx, double vy, double vz, double ax, double ay, double az, boolean wakeUp) {
        java.nio.ByteBuffer buffer = this.beginCommand(54);
        buffer.put((byte) 3).putInt(id);
        buffer.putDouble(vx).putDouble(vy).putDouble(vz);
        buffer.putDouble(ax).putDouble(ay).putDouble(az);
        buffer.put((byte) (wakeUp ? 1 : 0));
    }

    public void queueWakeUp(int id) {
        this.beginCommand(5).put((byte) 4).putInt(id);
    }

    @Override
    public void tick(double timeStep) {
        this.flushCommands();
        this.step(timeStep, 1);
    }

    void flushCommands() {
        if (this.commandCount > 0) {
            this.commandBuffer.putInt(6, this.commandCount);
            this.commandBuffer.putInt(10, this.commandBuffer.position());
            Rapier3D.processCommands(this.scene.handle(), this.commandBuffer, this.commandBuffer.position());
            this.commandBuffer.clear();
            this.commandCount = 0;
        }
    }

    void step(double timeStep, int elapsedTicks) {
        Rapier3D.step(this.scene.handle(), timeStep, elapsedTicks);
    }

    boolean hasNativeActivity() {
        return Rapier3D.hasActiveBodies(this.scene.handle());
    }

    long ticksUntilNextWake() {
        return Rapier3D.ticksUntilNextScheduledBody(this.scene.handle());
    }

    long invalidateSchedule() {
        return ++this.scheduleGeneration;
    }

    long scheduleGeneration() {
        return this.scheduleGeneration;
    }

    long lastStepTick() {
        return this.lastStepTick;
    }

    void setLastStepTick(long tick) {
        this.lastStepTick = tick;
    }

    boolean canStepInParallel() {
        return Rapier3D.canStepInParallel(this.scene.handle());
    }

    @Override
    public void dispose() {
        Rapier3D.dispose(this.scene.handle());
    }
}
