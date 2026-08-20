package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.TimeCacheAccessor;
import net.minecraft.world.level.Level;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;

@Mixin(Level.class)
public abstract class LevelTimeStateMixin implements TimeCacheAccessor {
    @Unique private long worldengine$cachedDayTime = -1L;
    @Unique private float worldengine$cachedPartialTick = -1.0F;
    @Unique private float worldengine$cachedTimeOfDayValue;

    @Override public long worldengine$getCachedDayTime() { return this.worldengine$cachedDayTime; }
    @Override public void worldengine$setCachedDayTime(long time) { this.worldengine$cachedDayTime = time; }
    @Override public float worldengine$getCachedPartialTick() { return this.worldengine$cachedPartialTick; }
    @Override public void worldengine$setCachedPartialTick(float tick) { this.worldengine$cachedPartialTick = tick; }
    @Override public float worldengine$getCachedTimeOfDayValue() { return this.worldengine$cachedTimeOfDayValue; }
    @Override public void worldengine$setCachedTimeOfDayValue(float value) { this.worldengine$cachedTimeOfDayValue = value; }
}
