package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEngineStorageState;
import dev.ryanhcode.sable.sublevel.storage.region.SubLevelRegionFile;
import dev.ryanhcode.sable.sublevel.storage.region.SubLevelStorageFile;
import dev.ryanhcode.sable.sublevel.storage.serialization.SubLevelStorage;
import it.unimi.dsi.fastutil.longs.Long2ObjectLinkedOpenHashMap;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;

@Mixin(SubLevelStorage.class)
public abstract class SubLevelStorageMixin implements WorldEngineStorageState {
    @Shadow @Final private Long2ObjectLinkedOpenHashMap<SubLevelRegionFile> regionCache;
    @Shadow @Final private Long2ObjectLinkedOpenHashMap<SubLevelStorageFile> storageCache;

    @Override
    public boolean worldengine$hasPendingWrites() {
        if (this.regionCache.isEmpty() && this.storageCache.isEmpty()) return false;
        for (SubLevelStorageFile file : this.regionCache.values()) {
            if (((WorldEngineStorageState) file).worldengine$hasPendingWrites()) return true;
        }
        for (SubLevelStorageFile file : this.storageCache.values()) {
            if (((WorldEngineStorageState) file).worldengine$hasPendingWrites()) return true;
        }
        return false;
    }
}
