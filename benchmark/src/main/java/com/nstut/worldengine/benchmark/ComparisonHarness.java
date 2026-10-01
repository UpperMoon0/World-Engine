package com.nstut.worldengine.benchmark;

import com.google.gson.GsonBuilder;
import dev.ryanhcode.sable.api.SubLevelAssemblyHelper;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.BoundingBox3d;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.GameRules;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.ModList;
import net.neoforged.fml.common.Mod;
import net.neoforged.neoforge.common.NeoForge;
import net.neoforged.neoforge.event.server.ServerStartedEvent;
import net.neoforged.neoforge.event.tick.ServerTickEvent;
import org.joml.Vector3d;

import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** A benchmark-only mod. Neither engine's production sources depend on it. */
@Mod("we_benchmark")
public final class ComparisonHarness {
    private final com.sun.management.ThreadMXBean thread =
            (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
    private final com.sun.management.OperatingSystemMXBean os =
            (com.sun.management.OperatingSystemMXBean) ManagementFactory.getOperatingSystemMXBean();
    private final List<ServerSubLevel> bodies = new ArrayList<>();
    private final List<Map<String, Object>> samples = new ArrayList<>();
    private final Map<String, Object> result = new LinkedHashMap<>();
    private final String engine = System.getProperty("we.benchmark.engine");
    private final String scenario = System.getenv("WE_BENCH_SCENARIO");
    private final int warmup = Integer.parseInt(System.getenv("WE_BENCH_WARMUP"));
    private final int measured = Integer.parseInt(System.getenv("WE_BENCH_TICKS"));
    private ServerLevel level;
    private int ticks;
    private boolean ready;
    private long start, cpu, allocated, processCpuStart;
    private double[] initialY;
    private double[] previousY;
    private int movingBodyObservations;

    public ComparisonHarness() {
        NeoForge.EVENT_BUS.register(this);
    }

    @SubscribeEvent
    public void started(ServerStartedEvent event) throws Exception {
        level = event.getServer().overworld();
        String provider = PhysicsPipelineProvider.INSTANCE.getClass().getName();
        boolean addon = "worldengine".equals(engine);
        require(ModList.get().isLoaded("worldengine") == addon, "Incorrect mod set");
        require(provider.equals(addon
                ? "com.nstut.worldengine.physics.rapier.RapierPhysicsPipelineProvider"
                : "dev.ryanhcode.sable.physics.impl.rapier.RapierPhysicsPipelineProvider"),
                "Incorrect physics provider: " + provider);
        if (!addon) {
            require(getClass().getClassLoader().getResource("worldengine.mixins.json") == null,
                    "Baseline contaminated with World Engine mixins");
        }
        require(thread.isThreadCpuTimeSupported() && thread.isThreadAllocatedMemorySupported(),
                "CPU/allocation counters unavailable");
        thread.setThreadCpuTimeEnabled(true);
        thread.setThreadAllocatedMemoryEnabled(true);
        require(os.getProcessCpuTime() >= 0, "Process CPU counter unavailable");
        level.getGameRules().getRule(GameRules.RULE_DOMOBSPAWNING).set(false, event.getServer());
        level.getGameRules().getRule(GameRules.RULE_RANDOMTICKING).set(0, event.getServer());
        level.getGameRules().getRule(GameRules.RULE_WEATHER_CYCLE).set(false, event.getServer());
        level.getGameRules().getRule(GameRules.RULE_DAYLIGHT).set(false, event.getServer());
        SubLevelPhysicsSystem system = SubLevelPhysicsSystem.require(level);
        require(!system.getPaused(), "Physics paused");
        int count = switch (scenario) {
            case "idle" -> 0;
            case "supported64", "active64", "edits64" -> 64;
            case "supported256" -> 256;
            default -> throw new IllegalArgumentException("Unknown fixture " + scenario);
        };
        // Fixed flat-world floor: y=-60 is the top of the grass layer.
        // A separate support layer is common to both engines.
        for (int x = 0; x < 72; x++) {
            for (int z = 0; z < 72; z++) {
                level.setBlock(new BlockPos(x, -59, z), Blocks.STONE.defaultBlockState(), 3);
            }
        }
        for (int x = -1; x <= 5; x++) for (int z = -1; z <= 5; z++) level.setChunkForced(x, z, true);
        for (int i = 0; i < count; i++) {
            BlockPos pos = new BlockPos(4 + (i % 16) * 4, -56, 4 + (i / 16) * 4);
            List<BlockPos> blocks = new ArrayList<>();
            for (int x = 0; x < 2; x++) for (int z = 0; z < 2; z++) {
                BlockPos block = pos.offset(x, 0, z);
                level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
                blocks.add(block);
            }
            bodies.add(SubLevelAssemblyHelper.assembleBlocks(level, pos, blocks,
                    new BoundingBox3i(pos.getX(), pos.getY(), pos.getZ(),
                            pos.getX() + 1, pos.getY(), pos.getZ() + 1)));
        }
        initialY = bodies.stream().mapToDouble(body -> body.logicalPose().position().y()).toArray();
        previousY = initialY.clone();
        result.put("schema", 1);
        result.put("runId", System.getenv("WE_BENCH_RUN"));
        result.put("engine", engine);
        result.put("scenario", scenario);
        result.put("provider", provider);
        result.put("bodies", count);
        result.put("blocksPerBody", 4);
        result.put("warmupTicks", warmup);
        result.put("measuredTicks", measured);
        result.put("physicsConfig", new GsonBuilder().create().toJsonTree(system.getConfig()));
        if (addon) result.put("submittedSceneSettingsAtStart", submittedSceneSettings());
        result.put("java", System.getProperty("java.runtime.version"));
        result.put("vm", System.getProperty("java.vm.name"));
        result.put("jvmArgs", ManagementFactory.getRuntimeMXBean().getInputArguments());
        result.put("os", System.getProperty("os.name") + " " + System.getProperty("os.version"));
        result.put("processors", os.getAvailableProcessors());
        result.put("maxHeapBytes", Runtime.getRuntime().maxMemory());
        result.put("mods", ModList.get().getMods().stream()
                .map(mod -> mod.getModId() + ":" + mod.getVersion()).sorted().toList());
        ready = true;
    }

    @SubscribeEvent
    public void pre(ServerTickEvent.Pre event) {
        if (!ready) return;
        // Equivalent workload commands are deliberately outside the tick timing interval.
        if ("active64".equals(scenario)) {
            for (int i = 0; i < bodies.size(); i++) {
                ServerSubLevel body = bodies.get(i);
                double y = body.logicalPose().position().y();
                if (ticks >= warmup && Math.abs(y - previousY[i]) > 0.01) movingBodyObservations++;
                previousY[i] = y;
                if (ticks % 10 == 0) SubLevelPhysicsSystem.require(level).getPhysicsHandle(body)
                        .addLinearAndAngularVelocity(new Vector3d(0, 2.0, 0), new Vector3d());
            }
        }
        if ("edits64".equals(scenario) && ticks % 10 == 0) {
            // Production terrain dirtying while retaining the support under every body.
            for (int i = 0; i < 64; i++) level.setBlock(
                    new BlockPos(3 + (i % 16) * 4, -58, 3 + (i / 16) * 4),
                    (ticks % 20 == 0 ? Blocks.STONE : Blocks.AIR).defaultBlockState(), 3);
        }
        if (ticks == warmup) processCpuStart = os.getProcessCpuTime();
        allocated = thread.getThreadAllocatedBytes(Thread.currentThread().threadId());
        cpu = thread.getCurrentThreadCpuTime();
        start = System.nanoTime();
    }

    @SubscribeEvent
    public void post(ServerTickEvent.Post event) throws Exception {
        if (!ready) return;
        long elapsed = System.nanoTime() - start;
        long cpuDelta = thread.getCurrentThreadCpuTime() - cpu;
        long allocationDelta = thread.getThreadAllocatedBytes(Thread.currentThread().threadId()) - allocated;
        if (ticks >= warmup) {
            require(elapsed >= 0 && cpuDelta >= 0 && allocationDelta >= 0, "Invalid counters");
            samples.add(Map.of("tick", ticks - warmup, "tickMs", elapsed / 1e6,
                    "serverThreadCpuMs", cpuDelta / 1e6, "serverThreadAllocatedBytes", allocationDelta));
        }
        ticks++;
        if (ticks == warmup + measured) {
            long processCpu = os.getProcessCpuTime() - processCpuStart;
            result.put("samples", samples);
            result.put("processCpuMs", processCpu / 1e6);
            result.put("initialY", initialY);
            if ("worldengine".equals(engine)) {
                result.put("submittedSceneSettingsAtEnd", submittedSceneSettings());
            }
            result.put("finalPoses", bodies.stream().map(body -> Map.of(
                    "x", body.logicalPose().position().x(), "y", body.logicalPose().position().y(),
                    "z", body.logicalPose().position().z(), "boundsMinY", body.boundingBox().minY(),
                    "removed", body.isRemoved())).toList());
            result.put("gcCollections", ManagementFactory.getGarbageCollectorMXBeans().stream()
                    .map(bean -> Map.of("name", bean.getName(), "count", bean.getCollectionCount(),
                            "timeMs", bean.getCollectionTime())).toList());
            try {
                verify();
                result.put("correctnessPass", true);
            } catch (IllegalStateException failure) {
                result.put("correctnessPass", false);
                result.put("correctnessError", failure.getMessage());
            }
            Path output = Path.of(System.getenv("WE_BENCH_OUTPUT"));
            Files.createDirectories(output.getParent());
            Files.writeString(output, new GsonBuilder().setPrettyPrinting().create().toJson(result));
            ready = false;
            event.getServer().halt(false);
        }
    }

    private void verify() {
        require(SubLevelContainer.getContainer(level).getAllSubLevels().size() == bodies.size(),
                "Body population changed");
        for (int i = 0; i < bodies.size(); i++) {
            ServerSubLevel body = bodies.get(i);
            var position = body.logicalPose().position();
            require(!body.isRemoved() && Double.isFinite(position.x()) && Double.isFinite(position.y())
                    && Double.isFinite(position.z()), "Missing/non-finite body");
            require(position.y() >= initialY[i] - 4 && position.y() <= initialY[i] + 4,
                    "Body " + i + " escaped terrain support: initialY=" + initialY[i]
                            + ", final=" + position);
            if (!"active64".equals(scenario)) {
                require(Math.abs(position.y() + 57.5) < 0.15 && position.y() < initialY[i] - 1.5,
                        "Body did not fall and settle on the known support height");
            }
            // Four blocks and valid collision geometry must survive the real pipeline.
            require(body.getMassTracker() != null && !body.getMassTracker().isInvalid(), "Invalid body mass");
        }
        // Compare production queries to an independent exact AABB oracle.
        int queryChecks = 0;
        for (int x = -4; x < 72; x += 4) {
            BoundingBox3d bounds = new BoundingBox3d(x, -64, 0, x + 6, -48, 72);
            Set<SubLevel> expected = new HashSet<>(), actual = new HashSet<>();
            for (SubLevel body : bodies) if (body.boundingBox().intersects(bounds)) expected.add(body);
            for (SubLevel body : SubLevelPhysicsSystem.require(level).queryIntersecting(bounds)) actual.add(body);
            require(actual.equals(expected), "Spatial query differs from exact AABB oracle");
            queryChecks++;
        }
        if ("active64".equals(scenario)) require(movingBodyObservations >= bodies.size(),
                "Active fixture did not show real body motion during measurement");
        result.put("movingBodyObservations", movingBodyObservations);
        result.put("queryChecks", queryChecks);
    }

    private Object submittedSceneSettings() throws ReflectiveOperationException {
        // Reflection keeps the stock source set free of addon classes. This is
        // setter-completion evidence, not a native readback API.
        Object pipeline = SubLevelPhysicsSystem.require(level).getPipeline();
        return pipeline.getClass().getMethod("worldengine$appliedSolverSettings").invoke(pipeline);
    }

    private static void require(boolean condition, String message) {
        if (!condition) throw new IllegalStateException(message);
    }
}
