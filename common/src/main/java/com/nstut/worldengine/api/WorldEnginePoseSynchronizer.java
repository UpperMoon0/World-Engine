package com.nstut.worldengine.api;

import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;

/** Implemented by addon pipelines capable of changed-body batch synchronization. */
public interface WorldEnginePoseSynchronizer {
    void worldengine$syncActivePoses(ServerSubLevelContainer container, WorldEnginePhysicsSystem system);
}
