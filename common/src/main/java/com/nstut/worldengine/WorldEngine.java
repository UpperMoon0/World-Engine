package com.nstut.worldengine;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class WorldEngine {
    public static final String MOD_ID = "worldengine";
    public static final Logger LOGGER = LoggerFactory.getLogger("World Engine");

    private WorldEngine() { }

    public static void initialize() {
        LOGGER.info("World Engine is replacing Sable's default physics pipeline with world-scale Rapier regions");
    }
}
