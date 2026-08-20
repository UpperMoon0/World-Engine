package com.nstut.worldengine.api;

public interface TimeCacheAccessor {
    long worldengine$getCachedDayTime();
    void worldengine$setCachedDayTime(long time);
    float worldengine$getCachedPartialTick();
    void worldengine$setCachedPartialTick(float tick);
    float worldengine$getCachedTimeOfDayValue();
    void worldengine$setCachedTimeOfDayValue(float value);
}
