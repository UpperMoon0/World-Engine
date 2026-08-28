package com.nstut.worldengine.physics;

import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Param;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.infra.Blackhole;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Opt-in crossover benchmark for Sable's linear scan, section lookup, and the
 * adaptive production policy. Run with {@code :common:jmh}; normal CI excludes it.
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.NANOSECONDS)
public class SectionSpatialIndexBenchmark {
    @State(Scope.Thread)
    public static class Workload {
        @Param({"1", "4", "8", "16", "32", "64", "128", "512"})
        int bodyCount;

        @Param({"1", "8", "64", "512", "4096"})
        int querySections;

        final List<Integer> bodies = new ArrayList<>();
        SectionSpatialIndex<Integer> index;

        @Setup(Level.Trial)
        public void setup() {
            this.index = new SectionSpatialIndex<>((x, y, z) -> ((long) x << 42)
                    ^ ((long) z << 20) ^ y);
            for (int body = 0; body < this.bodyCount; body++) {
                this.bodies.add(body);
                LongOpenHashSet sections = new LongOpenHashSet();
                sections.add(((long) body << 42));
                this.index.insert(body, sections);
            }
        }
    }

    @Benchmark
    public void sableLinearScan(Workload workload, Blackhole blackhole) {
        for (Integer body : workload.bodies) blackhole.consume(body == 0);
    }

    @Benchmark
    public void sectionIndex(Workload workload, Blackhole blackhole) {
        blackhole.consume(workload.index.querySections(0, 0, 0,
                workload.querySections - 1, 0, 0, body -> body == 0));
    }

    @Benchmark
    public void adaptiveHybrid(Workload workload, Blackhole blackhole) {
        if (workload.querySections <= (long) workload.bodyCount * 8L
                && workload.querySections <= 4096) {
            sectionIndex(workload, blackhole);
        } else {
            blackhole.consume(workload.index.queryAll(body -> body == 0));
        }
    }
}
