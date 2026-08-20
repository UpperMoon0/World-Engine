package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEnginePhysicsSystem;
import com.nstut.worldengine.api.WorldEnginePoseSynchronizer;
import com.nstut.worldengine.api.WorldEngineSubLevelActivity;
import com.nstut.worldengine.physics.WorldEngineBodyIndex;
import dev.ryanhcode.sable.ActiveSableCompanion;
import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.block.BlockEntitySubLevelActor;
import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.handle.RigidBodyHandle;
import dev.ryanhcode.sable.api.physics.object.ArbitraryPhysicsObject;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import dev.ryanhcode.sable.platform.SableEventPublishPlatform;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.storage.SubLevelRemovalReason;
import dev.ryanhcode.sable.mixinterface.plot.SubLevelContainerHolder;
import dev.ryanhcode.sable.sublevel.plot.LevelPlot;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.sublevel.system.ticket.PhysicsChunkTicketManager;
import it.unimi.dsi.fastutil.objects.ReferenceOpenHashSet;
import net.minecraft.CrashReport;
import net.minecraft.CrashReportCategory;
import net.minecraft.ReportedException;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.core.SectionPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import org.joml.Math;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.Redirect;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

@Mixin(value = SubLevelPhysicsSystem.class, priority = 1100)
public abstract class SubLevelPhysicsSystemMixin implements WorldEnginePhysicsSystem {
    @Shadow @Final private PhysicsPipeline pipeline;
    @Shadow @Final private ServerLevel level;
    @Shadow @Final private PhysicsConfigData config;
    @Shadow @Final private PhysicsChunkTicketManager ticketManager;
    @Shadow @Final private Pose3d storagePose;
    @Shadow @Final private Collection<ArbitraryPhysicsObject> queuedWakeUps;
    @Shadow private boolean paused;
    @Shadow private int currentSubstep;
    @Shadow private void tickPunchCooldowns() { }
    @Shadow public abstract double getPartialPhysicsTick();
    @Shadow public abstract RigidBodyHandle getPhysicsHandle(ServerSubLevel subLevel);
    @Shadow public abstract void updatePose(ServerSubLevel subLevel);
    @Shadow public abstract boolean recoverSubLevel(ServerSubLevel subLevel);

    @Unique private final Collection<ServerSubLevel> worldengine$active = new ReferenceOpenHashSet<>();
    @Unique private final Collection<ServerSubLevel> worldengine$nextActive = new ReferenceOpenHashSet<>();
    @Unique private final Collection<ServerSubLevel> worldengine$continuous = new ReferenceOpenHashSet<>();
    @Unique private List<ServerSubLevel> worldengine$activeSnapshot = List.of();
    @Unique private boolean worldengine$snapshotDirty = true;
    @Unique private final WorldEngineBodyIndex worldengine$bodyIndex = new WorldEngineBodyIndex();

    @Inject(method = "onSubLevelAdded", at = @At("TAIL"))
    private void worldengine$activateAdded(SubLevel subLevel, CallbackInfo ci) {
        if (subLevel instanceof ServerSubLevel serverSubLevel) this.worldengine$activate(serverSubLevel);
    }

    @Inject(method = "onSubLevelRemoved", at = @At("HEAD"))
    private void worldengine$forgetRemoved(SubLevel subLevel, SubLevelRemovalReason reason, CallbackInfo ci) {
        if (subLevel instanceof ServerSubLevel serverSubLevel) {
            this.worldengine$active.remove(serverSubLevel);
            this.worldengine$nextActive.remove(serverSubLevel);
            this.worldengine$continuous.remove(serverSubLevel);
            this.worldengine$bodyIndex.remove(serverSubLevel);
            this.worldengine$snapshotDirty = true;
        }
    }

    @Inject(method = "tick", at = @At("HEAD"), cancellable = true)
    private void worldengine$tickActiveBodies(SubLevelContainer sidelessContainer, CallbackInfo ci) {
        ci.cancel();
        ServerSubLevelContainer container = (ServerSubLevelContainer) sidelessContainer;
        this.tickPunchCooldowns();
        this.ticketManager.update(this.level, container, (SubLevelPhysicsSystem) (Object) this, this.pipeline, 1.0 / 20.0);

        for (ServerSubLevel subLevel : this.worldengine$activeBodies()) {
            if (subLevel.isRemoved()) continue;
            this.worldengine$refreshContinuousActivity(subLevel);
            subLevel.updateLastPose();
            for (BlockEntitySubLevelActor actor : subLevel.getPlot().getBlockEntityActors()) {
                actor.sable$tick(subLevel);
            }
        }

        this.pipeline.tick();
        if (this.paused) return;

        SubLevelPhysicsSystem.currentlySteppingSystem = (SubLevelPhysicsSystem) (Object) this;
        try {
            this.worldengine$tickPipelinePhysics(container);
        } catch (Exception exception) {
            CrashReport report = CrashReport.forThrowable(exception, "World Engine ticking Sable physics");
            CrashReportCategory category = report.addCategory("Current physics state");
            category.setDetail("Dimension", this.level.dimension());
            throw new ReportedException(report);
        } finally {
            SubLevelPhysicsSystem.currentlySteppingSystem = null;
            SubLevelPhysicsSystem.IN_PHYSICS_STEP = false;
        }
    }

    @Unique
    private void worldengine$tickPipelinePhysics(ServerSubLevelContainer container) {
        this.config.substepsPerTick = 1;
        this.pipeline.prePhysicsTicks();

        for (this.currentSubstep = 0; this.currentSubstep < this.config.substepsPerTick; this.currentSubstep++) {
            double timeStep = 1.0 / 20.0 / this.config.substepsPerTick;
            List<ServerSubLevel> physicsBodies = new ArrayList<>(this.worldengine$activeBodies());

            for (ServerSubLevel subLevel : physicsBodies) if (!subLevel.isRemoved()) subLevel.prePhysicsTickBegin();
            for (ServerSubLevel subLevel : physicsBodies) if (!subLevel.isRemoved()) subLevel.updateMergedMassData((float) this.getPartialPhysicsTick());
            for (ServerSubLevel subLevel : physicsBodies) if (!subLevel.isRemoved()) subLevel.prePhysicsTick(
                    (SubLevelPhysicsSystem) (Object) this, this.getPhysicsHandle(subLevel), timeStep);

            SableEventPublishPlatform.INSTANCE.prePhysicsTick((SubLevelPhysicsSystem) (Object) this, timeStep);
            for (ServerSubLevel subLevel : physicsBodies) if (!subLevel.isRemoved()) subLevel.applyQueuedForces(
                    (SubLevelPhysicsSystem) (Object) this, this.getPhysicsHandle(subLevel), timeStep);

            SubLevelPhysicsSystem.IN_PHYSICS_STEP = true;
            try {
                this.pipeline.physicsTick(timeStep);
            } finally {
                SubLevelPhysicsSystem.IN_PHYSICS_STEP = false;
            }

            container.processSubLevelRemovals();
            this.worldengine$updateAllPoses(container);
            for (ArbitraryPhysicsObject object : this.queuedWakeUps) object.wakeUp();
            this.queuedWakeUps.clear();
            SableEventPublishPlatform.INSTANCE.postPhysicsTick((SubLevelPhysicsSystem) (Object) this, timeStep);
        }

        this.pipeline.postPhysicsTicks();
        this.currentSubstep = this.config.substepsPerTick;
    }

    @Unique
    private void worldengine$updateAllPoses(ServerSubLevelContainer container) {
        if (this.pipeline instanceof WorldEnginePoseSynchronizer synchronizer) {
            synchronizer.worldengine$syncActivePoses(container, this);
            return;
        }
        this.worldengine$beginPoseSync();
        for (ServerSubLevel subLevel : this.worldengine$activeBodies()) {
            if (subLevel.isRemoved()) continue;
            this.updatePose(subLevel);
            this.worldengine$markActive(subLevel);
        }
        this.worldengine$endPoseSync();
    }

    @Override
    public Pose3d worldengine$storagePose() { return this.storagePose; }

    @Override
    public void worldengine$activate(ServerSubLevel subLevel) {
        if (subLevel.isRemoved()) return;
        this.worldengine$bodyIndex.update(subLevel);
        if (this.worldengine$active.add(subLevel)) this.worldengine$snapshotDirty = true;
        this.worldengine$nextActive.add(subLevel);
        this.worldengine$refreshContinuousActivity(subLevel);
    }

    @Override
    public List<ServerSubLevel> worldengine$activeBodies() {
        if (this.worldengine$snapshotDirty) {
            this.worldengine$activeSnapshot = List.copyOf(this.worldengine$active);
            this.worldengine$snapshotDirty = false;
        }
        return this.worldengine$activeSnapshot;
    }

    @Unique
    private void worldengine$refreshContinuousActivity(ServerSubLevel subLevel) {
        if (((WorldEngineSubLevelActivity) subLevel).worldengine$requiresContinuousPhysicsTick()) {
            this.worldengine$continuous.add(subLevel);
        } else {
            this.worldengine$continuous.remove(subLevel);
        }
    }

    @Override
    public void worldengine$beginPoseSync() {
        this.worldengine$nextActive.clear();
        this.worldengine$continuous.removeIf(SubLevel::isRemoved);
        this.worldengine$nextActive.addAll(this.worldengine$continuous);
    }

    @Override
    public void worldengine$markActive(ServerSubLevel subLevel) {
        if (!subLevel.isRemoved()) {
            this.worldengine$bodyIndex.update(subLevel);
            this.worldengine$nextActive.add(subLevel);
        }
    }

    @Override
    public void worldengine$endPoseSync() {
        if (!this.worldengine$active.equals(this.worldengine$nextActive)) {
            this.worldengine$active.clear();
            this.worldengine$active.addAll(this.worldengine$nextActive);
            this.worldengine$snapshotDirty = true;
        }
    }

    @Override
    public void worldengine$applyStoragePose(ServerSubLevel subLevel) {
        Vector3d position = this.storagePose.position();
        Quaterniond orientation = this.storagePose.orientation();
        if (Double.isNaN(position.x) || Double.isNaN(position.y) || Double.isNaN(position.z)
                || Double.isNaN(orientation.x) || Double.isNaN(orientation.y)
                || Double.isNaN(orientation.z) || Double.isNaN(orientation.w)) {
            Sable.LOGGER.info("Invalid position {} or orientation {} received for sub-level {} from World Engine.",
                    position, orientation, subLevel);
            if (!this.recoverSubLevel(subLevel)) return;
            this.pipeline.readPose(subLevel, this.storagePose);
        }

        Pose3d logicalPose = subLevel.logicalPose();
        logicalPose.position().set(this.storagePose.position());
        logicalPose.orientation().set(this.storagePose.orientation());
        logicalPose.position().sub(subLevel.lastPose().position(), subLevel.latestLinearVelocity);
        Quaterniond difference = logicalPose.orientation().difference(subLevel.lastPose().orientation(), new Quaterniond()).conjugate();
        Vector3d angularVelocity = subLevel.latestAngularVelocity.set(difference.x, difference.y, difference.z);
        if (angularVelocity.lengthSquared() <= 1E-15) angularVelocity.mul(2.0 / difference.w);
        else angularVelocity.normalize().mul(2.0 * Math.safeAcos(difference.w));
        subLevel.latestLinearVelocity.mul(20.0);
        subLevel.latestAngularVelocity.mul(20.0);
    }

    @Inject(method = "updateMassDataFromBlockChange", at = @At("TAIL"))
    private void worldengine$activateChangedMass(SubLevel subLevel, BlockPos pos, BlockState oldState,
            BlockState newState, boolean notifyPipeline, CallbackInfo ci) {
        if (subLevel instanceof ServerSubLevel serverSubLevel) this.worldengine$activate(serverSubLevel);
    }

    @Inject(method = "queryIntersecting", at = @At("HEAD"), cancellable = true)
    private void worldengine$queryTicketIndex(BoundingBox3dc bounds, CallbackInfoReturnable<Iterable<SubLevel>> cir) {
        cir.setReturnValue(this.worldengine$bodyIndex.query(bounds));
    }

    @Redirect(method = "handleBlockChange", at = @At(value = "INVOKE",
            target = "Ldev/ryanhcode/sable/ActiveSableCompanion;getContaining(Lnet/minecraft/world/level/Level;Lnet/minecraft/core/SectionPos;)Ldev/ryanhcode/sable/sublevel/SubLevel;"))
    private SubLevel worldengine$resolveChangedSubLevelDirectly(ActiveSableCompanion helper, Level level, SectionPos sectionPos) {
        LevelPlot plot = ((SubLevelContainerHolder) level).sable$getPlotContainer().getPlot(sectionPos.chunk());
        return plot == null ? null : plot.getSubLevel();
    }
}
