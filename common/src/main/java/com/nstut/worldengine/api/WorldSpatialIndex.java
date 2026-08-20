package com.nstut.worldengine.api;

import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import java.util.Collection;

/** Sparse world index that assigns Sable bodies to local physics regions. */
public interface WorldSpatialIndex {
    void addSubLevel(ServerSubLevel subLevel);
    void removeSubLevel(ServerSubLevel subLevel);
    Collection<PhysicsRegion> getRegions();
    void tick();
    void dispose();
}
