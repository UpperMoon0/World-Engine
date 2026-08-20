package com.nstut.worldengine.api;

import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import java.util.List;

/** Addon-owned bridge injected into Sable's physics system by Mixin. */
public interface WorldEnginePhysicsSystem {
    Pose3d worldengine$storagePose();
    void worldengine$activate(ServerSubLevel subLevel);
    List<ServerSubLevel> worldengine$activeBodies();
    void worldengine$beginPoseSync();
    void worldengine$markActive(ServerSubLevel subLevel);
    void worldengine$endPoseSync();
    void worldengine$applyStoragePose(ServerSubLevel subLevel);
}
