package com.nstut.worldengine.mixin;

import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import com.llamalad7.mixinextras.injector.wrapoperation.WrapOperation;
import com.llamalad7.mixinextras.sugar.Local;
import com.nstut.worldengine.api.WorldEngineClientInterpolation;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.network.client.ClientSableInterpolationState;
import dev.ryanhcode.sable.network.packets.ClientboundSableSnapshotDualPacket;
import dev.ryanhcode.sable.network.packets.PacketReceiveMode;
import dev.ryanhcode.sable.sublevel.ClientSubLevel;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;

@Mixin(ClientboundSableSnapshotDualPacket.class)
public abstract class SnapshotPacketMixin {
    @WrapOperation(method = "handleClient(Lnet/minecraft/world/level/Level;Ldev/ryanhcode/sable/network/packets/PacketReceiveMode;)V", at = @At(value = "INVOKE",
            target = "Ldev/ryanhcode/sable/network/client/ClientSableInterpolationState;receiveSnapshot"))
    private void worldengine$forwardVelocity(ClientSableInterpolationState state, ClientSubLevel subLevel,
            int gameTick, Pose3dc pose, PacketReceiveMode mode, Operation<Void> original,
            @Local ClientboundSableSnapshotDualPacket.Entry entry) {
        ((WorldEngineClientInterpolation) state).worldengine$receiveSnapshot(
                subLevel, gameTick, pose, entry.linearVelocity(), entry.angularVelocity(), mode);
    }
}
