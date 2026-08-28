package com.nstut.worldengine.physics;

import dev.ryanhcode.sable.companion.math.BoundingBox3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** Repeatable microbenchmark for the no-resident query path used by ordinary entity ticks. */
final class WorldEngineBodyIndexBenchmarkTest {
    private static final int WARMUP_ITERATIONS = 100_000;
    private static final int MEASURED_ITERATIONS = 2_000_000;
    private static volatile int blackhole;

    @Test
    void benchmarkEmptyIndexQueries() {
        WorldEngineBodyIndex index = new WorldEngineBodyIndex();

        benchmark(index, "single-section", new BoundingBox3d(1.0, 64.0, 1.0, 1.9, 65.8, 1.9));
        benchmark(index, "section-boundary", new BoundingBox3d(15.2, 63.2, 15.2, 16.8, 65.8, 16.8));
        benchmark(index, "512-sections", new BoundingBox3d(0.0, 0.0, 0.0, 127.0, 127.0, 127.0));
    }

    private static void benchmark(WorldEngineBodyIndex index, String name, BoundingBox3d bounds) {
        for (int i = 0; i < WARMUP_ITERATIONS; i++) consume(index.query(bounds));

        long started = System.nanoTime();
        for (int i = 0; i < MEASURED_ITERATIONS; i++) consume(index.query(bounds));
        long elapsed = System.nanoTime() - started;

        double nanosPerQuery = (double) elapsed / MEASURED_ITERATIONS;
        System.out.printf("WorldEngineBodyIndex %-16s %,10.2f ns/query (%d iterations)%n",
                name, nanosPerQuery, MEASURED_ITERATIONS);
        assertEquals(0, index.query(bounds).spliterator().getExactSizeIfKnown());
    }

    private static void consume(Iterable<?> value) {
        blackhole ^= System.identityHashCode(value);
    }
}
