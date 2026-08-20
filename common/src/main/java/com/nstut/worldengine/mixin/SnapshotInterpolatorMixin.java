package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEngineSnapshotInterpolator;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.network.client.SubLevelSnapshotInterpolator;
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ObjectArrayList;
import net.minecraft.util.Mth;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(SubLevelSnapshotInterpolator.class)
public abstract class SnapshotInterpolatorMixin implements WorldEngineSnapshotInterpolator {
    @Shadow @Final public ObjectArrayList<SubLevelSnapshotInterpolator.Snapshot> buffer;
    @Shadow private boolean stopped;
    @Shadow public abstract void receiveSnapshot(int gameTick, Pose3dc pose);
    @Unique private final Int2ObjectOpenHashMap<Vector3f> worldengine$linearVelocities = new Int2ObjectOpenHashMap<>();

    @Override
    public void worldengine$receiveSnapshot(int gameTick, Pose3dc pose, Vector3fc linearVelocity, Vector3fc angularVelocity) {
        this.receiveSnapshot(gameTick, pose);
        this.worldengine$linearVelocities.put(gameTick, new Vector3f(linearVelocity));
    }

    @Inject(method = "getSampleAt", at = @At("RETURN"))
    private void worldengine$extrapolateVelocity(double gameTick, Pose3d dest, CallbackInfo ci) {
        if (this.stopped || this.buffer.isEmpty()) return;
        SubLevelSnapshotInterpolator.Snapshot latest = this.buffer.getLast();
        if (latest.gameTick() >= gameTick) return;
        Vector3f velocity = this.worldengine$linearVelocities.get(latest.gameTick());
        if (velocity == null) return;
        double seconds = Mth.clamp(gameTick - latest.gameTick(), 0.0, 5.0) / 20.0;
        dest.set(latest.pose());
        dest.position().add(velocity.x * seconds, velocity.y * seconds, velocity.z * seconds);
    }

    @Inject(method = "tick", at = @At("HEAD"))
    private void worldengine$pruneVelocities(double backTick, CallbackInfo ci) {
        int oldest = (int) backTick - 8;
        this.worldengine$linearVelocities.keySet().removeIf(tick -> tick < oldest);
    }
}
