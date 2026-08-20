package com.nstut.worldengine.mixin;

import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.system.SubLevelTrackingSystem;
import com.nstut.worldengine.api.WorldEngineUpdateTicket;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Redirect;

import java.util.Collection;
import java.util.List;
import java.util.UUID;

@Mixin(SubLevelTrackingSystem.class)
public abstract class SubLevelTrackingSystemMixin {
    @Shadow @Final private ServerLevel level;
    @Shadow private int interpolationTick;

    @Redirect(method = "sendMovementUpdates", at = @At(value = "INVOKE",
            target = "Ljava/util/List;add(Ljava/lang/Object;)Z", ordinal = 0))
    private boolean worldengine$applyDistanceLod(List<Object> updates, Object value) {
        ServerSubLevel subLevel = (ServerSubLevel) ((WorldEngineUpdateTicket) value).worldengine$subLevel();
        if (!subLevel.getLastNetworkedStopped()) {
            int interval = this.worldengine$networkInterval(subLevel, subLevel.getTrackingPlayers());
            if (this.interpolationTick % interval != 0) return true;
        }
        return updates.add(value);
    }

    @Unique
    private int worldengine$networkInterval(ServerSubLevel subLevel, Collection<UUID> tracking) {
        double nearestDistanceSquared = Double.POSITIVE_INFINITY;
        var position = subLevel.logicalPose().position();
        for (UUID uuid : tracking) {
            ServerPlayer player = (ServerPlayer) this.level.getPlayerByUUID(uuid);
            if (player == null) continue;
            double dx = player.getX() - position.x();
            double dy = player.getY() - position.y();
            double dz = player.getZ() - position.z();
            nearestDistanceSquared = Math.min(nearestDistanceSquared, dx * dx + dy * dy + dz * dz);
        }
        if (nearestDistanceSquared <= 128.0 * 128.0) return 1;
        if (nearestDistanceSquared <= 512.0 * 512.0) return 2;
        return 5;
    }
}
