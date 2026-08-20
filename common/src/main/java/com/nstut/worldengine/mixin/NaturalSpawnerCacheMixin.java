package com.nstut.worldengine.mixin;

import net.minecraft.core.BlockPos;
import net.minecraft.core.Holder;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.util.random.WeightedRandomList;
import net.minecraft.world.entity.MobCategory;
import net.minecraft.world.level.NaturalSpawner;
import net.minecraft.world.level.StructureManager;
import net.minecraft.world.level.biome.Biome;
import net.minecraft.world.level.biome.MobSpawnSettings;
import net.minecraft.world.level.chunk.ChunkGenerator;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(value = NaturalSpawner.class, priority = 900)
public abstract class NaturalSpawnerCacheMixin {
    @Unique private static ServerLevel worldengine$level;
    @Unique private static long worldengine$gameTime = Long.MIN_VALUE;
    @Unique private static long worldengine$position = Long.MIN_VALUE;
    @Unique private static MobCategory worldengine$category;
    @Unique private static WeightedRandomList<MobSpawnSettings.SpawnerData> worldengine$result;

    @Inject(method = "mobsAt", at = @At("HEAD"), cancellable = true)
    private static void worldengine$getCachedMobs(ServerLevel level, StructureManager structures,
            ChunkGenerator generator, MobCategory category, BlockPos pos, Holder<Biome> biome,
            CallbackInfoReturnable<WeightedRandomList<MobSpawnSettings.SpawnerData>> cir) {
        long packed = pos == null ? 0L : pos.asLong();
        if (worldengine$level == level && worldengine$gameTime == level.getGameTime()
                && worldengine$position == packed && worldengine$category == category && worldengine$result != null) {
            cir.setReturnValue(worldengine$result);
        }
    }

    @Inject(method = "mobsAt", at = @At("RETURN"))
    private static void worldengine$rememberMobs(ServerLevel level, StructureManager structures,
            ChunkGenerator generator, MobCategory category, BlockPos pos, Holder<Biome> biome,
            CallbackInfoReturnable<WeightedRandomList<MobSpawnSettings.SpawnerData>> cir) {
        worldengine$level = level;
        worldengine$gameTime = level.getGameTime();
        worldengine$position = pos == null ? 0L : pos.asLong();
        worldengine$category = category;
        worldengine$result = cir.getReturnValue();
    }
}
