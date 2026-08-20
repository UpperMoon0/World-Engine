package com.nstut.worldengine.physics.rapier;

import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import net.minecraft.server.level.ServerLevel;
import org.jetbrains.annotations.NotNull;

@PhysicsPipelineProvider.LoadPriority(2000)
public final class RapierPhysicsPipelineProvider implements PhysicsPipelineProvider {

    @Override
    public @NotNull PhysicsPipeline createPipeline(@NotNull final ServerLevel level) {
        return new RapierPhysicsPipeline(level);
    }
}
