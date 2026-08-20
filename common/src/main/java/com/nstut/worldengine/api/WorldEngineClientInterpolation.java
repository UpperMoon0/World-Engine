package com.nstut.worldengine.api;

import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.network.packets.PacketReceiveMode;
import dev.ryanhcode.sable.sublevel.ClientSubLevel;
import org.joml.Vector3fc;

public interface WorldEngineClientInterpolation {
    void worldengine$receiveSnapshot(ClientSubLevel subLevel, int gameTick, Pose3dc pose,
            Vector3fc linearVelocity, Vector3fc angularVelocity, PacketReceiveMode mode);
}
