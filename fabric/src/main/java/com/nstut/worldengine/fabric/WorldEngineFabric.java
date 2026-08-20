package com.nstut.worldengine.fabric;

import com.nstut.worldengine.WorldEngine;
import net.fabricmc.api.ModInitializer;

public final class WorldEngineFabric implements ModInitializer {
    @Override
    public void onInitialize() {
        WorldEngine.initialize();
    }
}
