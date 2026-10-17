package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEngineStorageState;
import dev.ryanhcode.sable.sublevel.storage.region.SubLevelStorageFile;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/** Keep failed/partial writes pending until force() completes successfully. */
@Mixin(SubLevelStorageFile.class)
public abstract class SubLevelStorageFileMixin implements WorldEngineStorageState {
    @Unique private boolean worldengine$pendingWrites;

    @Inject(method = "<init>", at = @At("RETURN"))
    private void worldengine$initialFlush(CallbackInfo ci) {
        // Constructors can create/repair headers. Preserve their first flush.
        this.worldengine$pendingWrites = true;
    }

    @Inject(method = {"write(ILnet/minecraft/nbt/CompoundTag;)V", "write(ILjava/nio/ByteBuffer;)V", "writeHeader"},
            at = @At("HEAD"))
    private void worldengine$markWrite(CallbackInfo ci) {
        this.worldengine$pendingWrites = true;
    }

    @Inject(method = "flush", at = @At("HEAD"), cancellable = true)
    private void worldengine$skipCleanFlush(CallbackInfo ci) {
        if (!this.worldengine$pendingWrites) ci.cancel();
    }

    @Inject(method = "flush", at = @At("RETURN"))
    private void worldengine$flushed(CallbackInfo ci) {
        this.worldengine$pendingWrites = false;
    }

    @Override
    public boolean worldengine$hasPendingWrites() {
        return this.worldengine$pendingWrites;
    }
}
