package com.nstut.worldengine.api;

/** Internal persistence guard; a pending write must never be skipped. */
public interface WorldEngineStorageState {
    boolean worldengine$hasPendingWrites();
}
