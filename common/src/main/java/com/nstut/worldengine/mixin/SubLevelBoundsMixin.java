package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEnginePhysicsSystem;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(SubLevel.class)
public abstract class SubLevelBoundsMixin {
    @Inject(method = "updateBoundingBox", at = @At("TAIL"))
    private void worldengine$refreshExactQueryBounds(CallbackInfo ci) {
        if ((Object) this instanceof ServerSubLevel body && !body.isRemoved()) {
            SubLevelPhysicsSystem system = SubLevelPhysicsSystem.get(body.getLevel());
            if (system instanceof WorldEnginePhysicsSystem indexed) {
                // Bounds, rather than native activity, determine index freshness.
                // Refresh only registered entries; construction/removal never adds one.
                indexed.worldengine$refreshQueryBounds(body);
            }
        }
    }
}
