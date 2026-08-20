package com.nstut.worldengine.api;

import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import java.util.Collection;

/** A local physics interaction cluster owned by World Engine. */
public interface PhysicsRegion {
    long getSceneHandle();
    Collection<ServerSubLevel> getActiveSubLevels();
    void addSubLevel(ServerSubLevel subLevel);
    void removeSubLevel(ServerSubLevel subLevel);
    void tick(double timeStep);
    void dispose();
}
