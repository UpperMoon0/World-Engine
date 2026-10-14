package com.nstut.worldengine.mixin;

import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import com.llamalad7.mixinextras.injector.wrapoperation.WrapOperation;
import com.nstut.worldengine.api.WorldEngineStorageState;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.sublevel.storage.SubLevelTicketsSavedData;
import dev.ryanhcode.sable.sublevel.storage.holding.GlobalSavedSubLevelPointer;
import dev.ryanhcode.sable.sublevel.storage.holding.SubLevelHoldingChunk;
import dev.ryanhcode.sable.sublevel.storage.holding.SubLevelHoldingChunkMap;
import dev.ryanhcode.sable.sublevel.storage.serialization.SubLevelStorage;
import it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import it.unimi.dsi.fastutil.longs.LongSet;
import it.unimi.dsi.fastutil.objects.ObjectSet;
import net.minecraft.world.level.ChunkPos;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(SubLevelHoldingChunkMap.class)
public abstract class SubLevelHoldingChunkMapMixin {
    @Shadow @Final private ServerSubLevelContainer container;
    @Shadow @Final private SubLevelStorage storage;
    @Shadow @Final private Long2ObjectMap<SubLevelHoldingChunk> loadedHoldingChunks;
    @Shadow @Final private LongSet dirtyHoldingChunks;
    @Shadow @Final private ObjectSet<ChunkPos> queuedUnloads;
    @Shadow @Final private ObjectSet<GlobalSavedSubLevelPointer> queuedDeletion;
    @Shadow @Final private LongSet chunksToUnload;
    @Shadow @Final private LongSet chunksToLoad;

    @Inject(method = "saveAll", at = @At("HEAD"), cancellable = true)
    private void worldengine$skipEmptySave(CallbackInfo ci) {
        if (this.container.getAllSubLevels().isEmpty() && this.loadedHoldingChunks.isEmpty()
                && this.dirtyHoldingChunks.isEmpty() && this.queuedUnloads.isEmpty()
                && this.queuedDeletion.isEmpty() && this.chunksToUnload.isEmpty()
                && this.chunksToLoad.isEmpty()
                && !((WorldEngineStorageState) this.storage).worldengine$hasPendingWrites()) {
            ci.cancel();
        }
    }

    @Inject(method = "processChanges", at = @At("HEAD"), cancellable = true)
    private void worldengine$skipEmptyProcessing(CallbackInfo ci) {
        // Loaded holding chunks must still test readiness, even with no new events.
        if (this.loadedHoldingChunks.isEmpty() && this.chunksToUnload.isEmpty()
                && this.chunksToLoad.isEmpty()) ci.cancel();
    }

    @WrapOperation(method = "saveAll", at = @At(value = "INVOKE",
            target = "Ldev/ryanhcode/sable/sublevel/storage/SubLevelTicketsSavedData;setDirty()V"))
    private void worldengine$dirtyTicketPointers(SubLevelTicketsSavedData data, Operation<Void> original) {
        // Ticket add/remove already marks SavedData dirty. Nonempty tickets also
        // need new serialization pointers; never clear an existing dirty flag.
        if (!this.container.getAllTickets().isEmpty()) original.call(data);
    }
}
