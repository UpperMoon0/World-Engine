package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.TimeCacheAccessor;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.LevelTimeAccess;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(value = LevelTimeAccess.class, priority = 900)
public interface LevelTimeAccessCacheMixin {
    @Inject(method = "getTimeOfDay(F)F", at = @At("HEAD"), cancellable = true)
    default void worldengine$getTimeOfDayCached(float partialTick, CallbackInfoReturnable<Float> cir) {
        if ((Object) this instanceof Level level) {
            TimeCacheAccessor cache = (TimeCacheAccessor) level;
            long dayTime = level.getLevelData() == null ? 0L : level.getLevelData().getDayTime();
            if (cache.worldengine$getCachedDayTime() == dayTime
                    && Float.compare(cache.worldengine$getCachedPartialTick(), partialTick) == 0) {
                cir.setReturnValue(cache.worldengine$getCachedTimeOfDayValue());
            }
        }
    }

    @Inject(method = "getTimeOfDay(F)F", at = @At("RETURN"))
    default void worldengine$rememberTimeOfDay(float partialTick, CallbackInfoReturnable<Float> cir) {
        if ((Object) this instanceof Level level) {
            TimeCacheAccessor cache = (TimeCacheAccessor) level;
            cache.worldengine$setCachedDayTime(level.getLevelData() == null ? 0L : level.getLevelData().getDayTime());
            cache.worldengine$setCachedPartialTick(partialTick);
            cache.worldengine$setCachedTimeOfDayValue(cir.getReturnValueF());
        }
    }
}
