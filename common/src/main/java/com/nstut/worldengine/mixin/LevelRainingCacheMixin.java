package com.nstut.worldengine.mixin;

import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(value = Level.class, priority = 900)
public abstract class LevelRainingCacheMixin {
    @Unique private long worldengine$rainGameTime = Long.MIN_VALUE;
    @Unique private long worldengine$rainPos = Long.MIN_VALUE;
    @Unique private boolean worldengine$rainValue;

    @Inject(method = "isRainingAt(Lnet/minecraft/core/BlockPos;)Z", at = @At("HEAD"), cancellable = true)
    private void worldengine$getCachedRain(BlockPos pos, CallbackInfoReturnable<Boolean> cir) {
        Level level = (Level) (Object) this;
        long gameTime = level.getGameTime();
        long packed = pos == null ? 0L : pos.asLong();
        if (this.worldengine$rainGameTime == gameTime && this.worldengine$rainPos == packed) {
            cir.setReturnValue(this.worldengine$rainValue);
        }
    }

    @Inject(method = "isRainingAt(Lnet/minecraft/core/BlockPos;)Z", at = @At("RETURN"))
    private void worldengine$rememberRain(BlockPos pos, CallbackInfoReturnable<Boolean> cir) {
        Level level = (Level) (Object) this;
        this.worldengine$rainGameTime = level.getGameTime();
        this.worldengine$rainPos = pos == null ? 0L : pos.asLong();
        this.worldengine$rainValue = cir.getReturnValueZ();
    }
}
