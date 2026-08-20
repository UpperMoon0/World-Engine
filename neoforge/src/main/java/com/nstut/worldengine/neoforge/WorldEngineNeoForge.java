package com.nstut.worldengine.neoforge;

import com.nstut.worldengine.WorldEngine;
import net.neoforged.fml.common.Mod;

@Mod(WorldEngine.MOD_ID)
public final class WorldEngineNeoForge {
    public WorldEngineNeoForge() {
        WorldEngine.initialize();
    }
}
