package com.nstut.worldengine.api;

import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import java.util.Map;

/** Diagnostics for settings successfully submitted through the native configuration setters. */
public interface WorldEngineSolverConfiguration {
    record Settings(double contactSpringFrequency, double contactSpringDampingRatio,
                    int solverIterations, int pgsIterations, int stabilizationIterations,
                    int minDynamicBodiesPerIsland) {
        public static Settings from(PhysicsConfigData data) {
            return new Settings(data.contactSpringFrequency, data.contactSpringDampingRatio,
                    data.solverIterations, data.pgsIterations, data.stabilizationIterations,
                    data.minDynamicBodiesPerIsland);
        }
    }

    Map<Long, Settings> worldengine$appliedSolverSettings();
}
