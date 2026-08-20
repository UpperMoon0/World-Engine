package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEnginePhysicsSystem;
import com.nstut.worldengine.api.WorldEngineSubLevelActivity;
import dev.ryanhcode.sable.api.physics.force.ForceGroup;
import dev.ryanhcode.sable.api.physics.force.QueuedForceGroup;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.plot.ServerLevelPlot;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(ServerSubLevel.class)
public abstract class ServerSubLevelActivityMixin implements WorldEngineSubLevelActivity {
    @Inject(method = "getOrCreateQueuedForceGroup", at = @At("HEAD"))
    private void worldengine$activateForForce(ForceGroup group, CallbackInfoReturnable<QueuedForceGroup> cir) {
        ServerSubLevel self = (ServerSubLevel) (Object) this;
        SubLevelPhysicsSystem system = SubLevelPhysicsSystem.get(self.getLevel());
        if (system instanceof WorldEnginePhysicsSystem optimized) optimized.worldengine$activate(self);
    }

    @Override
    public boolean worldengine$requiresContinuousPhysicsTick() {
        ServerSubLevel self = (ServerSubLevel) (Object) this;
        ServerLevelPlot plot = self.getPlot();
        return plot.getBlockEntityActors().iterator().hasNext()
                || !plot.getLiftProviders().isEmpty()
                || !plot.getContraptions().isEmpty()
                || self.getFloatingBlockController().needsTicking()
                || self.getReactionWheelManager().needsTicking();
    }
}
