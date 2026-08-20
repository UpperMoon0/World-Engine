package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEngineClientInterpolation;
import com.nstut.worldengine.api.WorldEngineSnapshotInterpolator;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.network.client.ClientSableInterpolationState;
import dev.ryanhcode.sable.network.packets.PacketReceiveMode;
import dev.ryanhcode.sable.sublevel.ClientSubLevel;
import org.joml.Vector3fc;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;

@Mixin(ClientSableInterpolationState.class)
public abstract class ClientInterpolationStateMixin implements WorldEngineClientInterpolation {
    @Shadow private PacketReceiveMode receivingMode;

    @Override
    public void worldengine$receiveSnapshot(ClientSubLevel subLevel, int gameTick, Pose3dc pose,
            Vector3fc linearVelocity, Vector3fc angularVelocity, PacketReceiveMode mode) {
        this.receivingMode = mode;
        ((WorldEngineSnapshotInterpolator) subLevel.getInterpolator()).worldengine$receiveSnapshot(
                gameTick, pose, linearVelocity, angularVelocity);
    }
}
