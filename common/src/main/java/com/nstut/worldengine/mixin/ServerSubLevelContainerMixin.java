package com.nstut.worldengine.mixin;

import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(ServerSubLevelContainer.class)
public abstract class ServerSubLevelContainerMixin {
    @Inject(method = "tick", at = @At("HEAD"), cancellable = true)
    private void worldengine$skipEmptyContainer(CallbackInfo ci) {
        ServerSubLevelContainer self = (ServerSubLevelContainer) (Object) this;
        if (self.getLoadedCount() == 0 && self.getAllSubLevels().isEmpty()) ci.cancel();
    }
}
