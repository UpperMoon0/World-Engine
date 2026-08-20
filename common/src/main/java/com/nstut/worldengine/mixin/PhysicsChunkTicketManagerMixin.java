package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEnginePhysicsSystem;
import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.sublevel.system.ticket.PhysicsChunkTicketManager;
import net.minecraft.server.level.ServerLevel;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.Redirect;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

import java.util.List;

@Mixin(PhysicsChunkTicketManager.class)
public abstract class PhysicsChunkTicketManagerMixin {
    @Unique private WorldEnginePhysicsSystem worldengine$physicsSystem;

    @Inject(method = "update", at = @At("HEAD"))
    private void worldengine$captureActiveSet(ServerLevel level, ServerSubLevelContainer container,
            SubLevelPhysicsSystem system, PhysicsPipeline pipeline, double timeStep, CallbackInfo ci) {
        this.worldengine$physicsSystem = (WorldEnginePhysicsSystem) system;
    }

    @Inject(method = "update", at = @At("RETURN"))
    private void worldengine$releaseActiveSet(ServerLevel level, ServerSubLevelContainer container,
            SubLevelPhysicsSystem system, PhysicsPipeline pipeline, double timeStep, CallbackInfo ci) {
        this.worldengine$physicsSystem = null;
    }

    @Redirect(method = "update", at = @At(value = "INVOKE",
            target = "Ldev/ryanhcode/sable/api/sublevel/ServerSubLevelContainer;getAllSubLevels()Ljava/util/List;"))
    private List<ServerSubLevel> worldengine$iterateActiveBodies(ServerSubLevelContainer container) {
        return this.worldengine$physicsSystem.worldengine$activeBodies();
    }
}
