package com.nstut.worldengine.api;

import dev.ryanhcode.sable.sublevel.ServerSubLevel;

import java.util.List;

/** Bodies whose collision terrain must survive even while the solver sleeps. */
public interface WorldEngineTerrainBodies {
    List<ServerSubLevel> worldengine$ticketBodies(List<ServerSubLevel> activeBodies);
}
