package com.nstut.worldengine.mixin;

import com.nstut.worldengine.api.WorldEngineUpdateTicket;
import dev.ryanhcode.sable.sublevel.SubLevel;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;

@Mixin(targets = "dev.ryanhcode.sable.sublevel.system.SubLevelTrackingSystem$SubLevelUpdateTicket")
public abstract class SubLevelUpdateTicketMixin implements WorldEngineUpdateTicket {
    @Shadow public abstract SubLevel subLevels();

    @Override
    public SubLevel worldengine$subLevel() {
        return this.subLevels();
    }
}
