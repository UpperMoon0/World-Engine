package com.nstut.worldengine.api;

import dev.ryanhcode.sable.companion.math.Pose3dc;
import org.joml.Vector3fc;

public interface WorldEngineSnapshotInterpolator {
    void worldengine$receiveSnapshot(int gameTick, Pose3dc pose, Vector3fc linearVelocity, Vector3fc angularVelocity);
}
