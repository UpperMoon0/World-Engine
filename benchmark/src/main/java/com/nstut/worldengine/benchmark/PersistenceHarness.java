package com.nstut.worldengine.benchmark;

import com.google.gson.GsonBuilder;
import dev.ryanhcode.sable.SableConfig;
import dev.ryanhcode.sable.api.SubLevelAssemblyHelper;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.ticket.SubLevelLoadingTicketType;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.storage.SubLevelRemovalReason;
import dev.ryanhcode.sable.sublevel.storage.SubLevelTicketsSavedData;
import dev.ryanhcode.sable.sublevel.storage.region.SubLevelStorageFile;
import dev.ryanhcode.sable.sublevel.storage.holding.GlobalSavedSubLevelPointer;
import net.minecraft.core.BlockPos;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.nbt.NbtIo;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.util.Unit;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.ChunkPos;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.ModList;
import net.neoforged.neoforge.event.server.ServerStartedEvent;
import net.neoforged.neoforge.event.tick.ServerTickEvent;

import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

/** Real server saves; no production classes depend on this opt-in fixture. */
public final class PersistenceHarness {
    private final com.sun.management.ThreadMXBean thread =
            (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
    private final com.sun.management.OperatingSystemMXBean os =
            (com.sun.management.OperatingSystemMXBean) ManagementFactory.getOperatingSystemMXBean();
    private final String engine = System.getProperty("we.benchmark.engine");
    private final String scenario = System.getenv("WE_BENCH_SCENARIO");
    private final String mode = System.getenv().getOrDefault("WE_PERSIST_MODE", "measure");
    private final int extraDimensions = Integer.parseInt(scenario.substring("dimensions".length()));
    private final int warmup = Integer.parseInt(System.getenv("WE_BENCH_WARMUP"));
    private final int measured = Integer.parseInt(System.getenv("WE_BENCH_TICKS"));
    private final List<Map<String, Object>> ticks = new ArrayList<>(), saves = new ArrayList<>();
    private final Map<String, Object> result = new LinkedHashMap<>();
    private List<ServerLevel> emptyDimensions;
    private int tick;
    private long start, cpu, allocation, processStart;
    private boolean ready;

    @SubscribeEvent
    public void started(ServerStartedEvent event) throws Exception {
        boolean addon = "worldengine".equals(engine);
        require(ModList.get().isLoaded("worldengine") == addon, "Wrong mod set");
        String provider = PhysicsPipelineProvider.INSTANCE.getClass().getName();
        require(provider.equals(addon ? "com.nstut.worldengine.physics.rapier.RapierPhysicsPipelineProvider"
                : "dev.ryanhcode.sable.physics.impl.rapier.RapierPhysicsPipelineProvider"), "Wrong provider");
        if (!addon) require(getClass().getClassLoader().getResource("worldengine.mixins.json") == null,
                "Baseline contains addon mixins");
        require(thread.isThreadCpuTimeSupported() && thread.isThreadAllocatedMemorySupported(), "Missing counters");
        thread.setThreadCpuTimeEnabled(true);
        thread.setThreadAllocatedMemoryEnabled(true);
        require(os.getProcessCpuTime() >= 0, "Missing process counter");
        emptyDimensions = new ArrayList<>();
        for (ServerLevel level : event.getServer().getAllLevels()) {
            if (level.dimension().location().getNamespace().equals("we_persistence")) emptyDimensions.add(level);
        }
        require(emptyDimensions.size() == extraDimensions, "Missing datapack dimensions");
        result.put("schema", 1);
        result.put("runId", System.getenv("WE_BENCH_RUN"));
        result.put("engine", engine);
        result.put("scenario", scenario);
        result.put("mode", mode);
        result.put("extraDimensions", extraDimensions);
        result.put("warmupTicks", warmup);
        result.put("measuredTicks", measured);
        result.put("saveIntervalTicks", 10);
        require(!SableConfig.SUB_LEVEL_SAVING_LOG_MESSAGE.get()
                && !SableConfig.VERBOSE_SERIALIZATION_LOGGING.get(), "Save logging must be disabled in both variants");
        result.put("saveLogging", false);
        result.put("java", System.getProperty("java.runtime.version"));
        result.put("os", System.getProperty("os.name"));
        result.put("processors", os.getAvailableProcessors());
        result.put("maxHeapBytes", Runtime.getRuntime().maxMemory());
        result.put("mods", ModList.get().getMods().stream()
                .map(mod -> mod.getModId() + ":" + mod.getVersion()).sorted().toList());
        ready = true;
    }

    @SubscribeEvent
    public void pre(ServerTickEvent.Pre event) {
        if (!ready) return;
        if (tick == warmup) processStart = os.getProcessCpuTime();
        allocation = thread.getThreadAllocatedBytes(Thread.currentThread().threadId());
        cpu = thread.getCurrentThreadCpuTime();
        start = System.nanoTime();
    }

    @SubscribeEvent
    public void post(ServerTickEvent.Post event) throws Exception {
        if (!ready) return;
        long elapsed = System.nanoTime() - start;
        long cpuDelta = thread.getCurrentThreadCpuTime() - cpu;
        long bytes = thread.getThreadAllocatedBytes(Thread.currentThread().threadId()) - allocation;
        if (!mode.equals("measure")) {
            try {
                restartCheck(event.getServer());
                result.put("correctnessPass", true);
            } catch (Exception | AssertionError failure) {
                result.put("correctnessPass", false);
                result.put("correctnessError", failure.toString());
            }
            finish(event.getServer());
            return;
        }
        if (tick >= warmup) ticks.add(sample(tick - warmup, elapsed, cpuDelta, bytes));
        if (tick % 10 == 0) {
            long saveBytes = thread.getThreadAllocatedBytes(Thread.currentThread().threadId());
            long saveCpu = thread.getCurrentThreadCpuTime();
            long saveStart = System.nanoTime();
            // Same production entry point as autosave, excluding player saves.
            event.getServer().saveAllChunks(true, false, false);
            long saveElapsed = System.nanoTime() - saveStart;
            long saveCpuDelta = thread.getCurrentThreadCpuTime() - saveCpu;
            long saveAllocation = thread.getThreadAllocatedBytes(Thread.currentThread().threadId()) - saveBytes;
            if (tick >= warmup) saves.add(sample(tick - warmup, saveElapsed, saveCpuDelta, saveAllocation));
        }
        if (++tick == warmup + measured) {
            result.put("processCpuMs", (os.getProcessCpuTime() - processStart) / 1e6);
            result.put("ticks", ticks);
            result.put("saves", saves);
            try {
                for (ServerLevel level : emptyDimensions) {
                    require(level.getChunkSource().getLoadedChunksCount() == 0, "Fixture loaded chunks");
                    ServerSubLevelContainer container = SubLevelContainer.getContainer(level);
                    require(container.getAllSubLevels().isEmpty(), "Fixture contains bodies");
                    require(container.getAllTickets().isEmpty(), "Fixture contains tickets");
                    if (engine.equals("worldengine")) require(!SubLevelTicketsSavedData.getOrLoad(level).isDirty(),
                            "Empty ticket file became dirty");
                }
                storageChecks(Path.of(System.getenv("WE_BENCH_OUTPUT")).getParent());
                prepareRestart(event.getServer());
                result.put("correctnessPass", true);
            } catch (Exception | AssertionError failure) {
                result.put("correctnessPass", false);
                result.put("correctnessError", failure.toString());
            }
            finish(event.getServer());
        }
    }

    private Map<String, Object> sample(int index, long ns, long cpuNs, long bytes) {
        require(ns >= 0 && cpuNs >= 0 && bytes >= 0, "Invalid counter");
        return Map.of("tick", index, "wallMs", ns / 1e6, "cpuMs", cpuNs / 1e6, "allocatedBytes", bytes);
    }

    private void storageChecks(Path root) throws Exception {
        Path file = root.resolve("storage-regression.slvl"), external = root.resolve("external-regression");
        CompoundTag tag = new CompoundTag();
        tag.putString("sentinel", "preserved");
        boolean closedChecksCompleted = false;
        try (SubLevelStorageFile storage = new SubLevelStorageFile(file, external)) {
            storage.write(0, tag);
            storage.flush();
            storage.flush();
            require(tag.equals(storage.read(0)), "Repeated flush lost data");
            // The public stream bypasses write(CompoundTag); its close still
            // reaches the protected byte-buffer write and must dirty the file.
            try (var output = storage.getSubLevelDataOutputStream(1)) {
                if (SubLevelStorageFile.COMPRESS_DATA) NbtIo.writeCompressed(tag, output);
                else NbtIo.write(tag, output);
            }
            if (engine.equals("worldengine")) require(Boolean.TRUE.equals(storage.getClass()
                    .getMethod("worldengine$hasPendingWrites").invoke(storage)), "Stream write was not tracked");
            storage.flush();
        }
        try (SubLevelStorageFile storage = new SubLevelStorageFile(file, external)) {
            require(tag.equals(storage.read(0)), "Reopen lost data");
            require(tag.equals(storage.read(1)), "Stream write lost after reopen");
            storage.write(0, (CompoundTag) null);
            storage.flush();
            require(storage.read(0) == null, "Deletion lost");
            if (engine.equals("worldengine")) {
                var pending = storage.getClass().getMethod("worldengine$hasPendingWrites");
                require(Boolean.FALSE.equals(pending.invoke(storage)), "Successful flush stayed pending");
                // Closed channels fail writes/force; neither exception may clear pending state.
                var channel = SubLevelStorageFile.class.getDeclaredField("file");
                channel.setAccessible(true);
                ((java.nio.channels.FileChannel) channel.get(storage)).close();
                storage.flush(); // A clean cached file must not call force().
                try { storage.write(0, tag); throw new AssertionError("Closed write succeeded"); }
                catch (java.io.IOException expected) { }
                try { storage.flush(); throw new AssertionError("Failed write was skipped"); }
                catch (java.io.IOException expected) { }
                require(Boolean.TRUE.equals(pending.invoke(storage)), "Failed flush cleared pending state");
                closedChecksCompleted = true;
                // close() deliberately still forces, so test it outside try-with-resources below.
            }
        } catch (java.nio.channels.ClosedChannelException expected) {
            require(engine.equals("worldengine") && closedChecksCompleted, "Unexpected closed-channel failure");
        }
        result.put("storageChecksPass", true);
    }

    private void prepareRestart(MinecraftServer server) throws Exception {
        if (!"true".equals(System.getenv("WE_PERSIST_RESTART_CHECK"))) return;
        ServerLevel level = server.overworld();
        BlockPos pos = new BlockPos(8, -56, 8);
        level.getChunk(0, 0);
        level.setChunkForced(0, 0, true);
        level.setBlock(pos, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, pos, List.of(pos),
                new BoundingBox3i(pos.getX(), pos.getY(), pos.getZ(), pos.getX(), pos.getY(), pos.getZ()));
        ServerSubLevelContainer container = SubLevelContainer.getContainer(level);
        container.addForceLoadTicket(body, SubLevelLoadingTicketType.COMMAND_FORCED, Unit.INSTANCE);
        server.saveAllChunks(true, true, false);
        require(body.getLastSerializationPointer() != null, "Missing saved pointer");
        UUID uuid = body.getUniqueId();
        container.removeForceLoadTicket(body, SubLevelLoadingTicketType.COMMAND_FORCED, Unit.INSTANCE);
        // Leave a real holding sublevel in an unloaded chunk with zero active
        // bodies. Its migration/save must not be mistaken for an empty world.
        body.logicalPose().position().set(100008, -56, 100008);
        body.updateBoundingBox();
        ChunkPos holdingPos = new ChunkPos(6250, 6250);
        var holding = container.getHoldingChunkMap();
        holding.moveToUnloaded(body, holdingPos);
        require(container.getAllSubLevels().isEmpty(), "Unload did not remove active body");
        server.saveAllChunks(true, true, false);
        require(container.getAllSubLevels().isEmpty(), "Holding fixture unexpectedly reloaded during save");
        var savedHolding = holding.getStorage().attemptLoadHoldingChunk(holdingPos);
        require(savedHolding != null && savedHolding.getSubLevelPointers().size() == 1, "Pending holding save lost");
        var local = savedHolding.getSubLevelPointers().getFirst();
        var pointer = new GlobalSavedSubLevelPointer(holdingPos, local.storageIndex(), local.subLevelIndex());
        require(holding.getStorage().attemptLoadSubLevel(holdingPos, local).uuid().equals(uuid), "Wrong saved holding body");
        holding.snatchAndLoad(pointer, uuid);
        body = (ServerSubLevel) container.getSubLevel(uuid);
        require(body != null, "Holding reload lost body");
        body.logicalPose().position().set(8, -56, 8);
        body.updateBoundingBox();
        container.addForceLoadTicket(body, SubLevelLoadingTicketType.COMMAND_FORCED, Unit.INSTANCE);
        server.saveAllChunks(true, true, false);
        require(body.getLastSerializationPointer() != null, "Reloaded body lost save pointer");
        result.put("holdingUnloadReloadPass", true);
        Files.writeString(Path.of(System.getenv("WE_BENCH_OUTPUT")).getParent().resolve("restart-uuid.txt"),
                uuid.toString());
    }

    private void restartCheck(MinecraftServer server) throws Exception {
        UUID uuid = UUID.fromString(Files.readString(Path.of(System.getenv("WE_BENCH_OUTPUT"))
                .getParent().resolve("restart-uuid.txt")).trim());
        ServerSubLevelContainer container = SubLevelContainer.getContainer(server.overworld());
        ServerSubLevel body = (ServerSubLevel) container.getSubLevel(uuid);
        if (mode.equals("deleted")) {
            require(body == null && !container.getAllTickets().containsKey(uuid), "Deleted body/ticket returned");
        } else {
            require(body != null && body.getLastSerializationPointer() != null, "Force-loaded body missing after restart");
            require(!body.getMassTracker().isInvalid(), "Invalid body mass after restart");
            var pointer = body.getLastSerializationPointer();
            require(container.removeForceLoadTicket(body, SubLevelLoadingTicketType.COMMAND_FORCED, Unit.INSTANCE),
                    "Force-load ticket missing");
            container.removeSubLevel(body, SubLevelRemovalReason.REMOVED);
            require(container.getAllSubLevels().isEmpty(), "Deletion fixture is not empty");
            server.saveAllChunks(true, true, false);
            require(container.getHoldingChunkMap().getStorage().attemptLoadSubLevel(pointer.chunkPos(), pointer.local()) == null,
                    "Pending deletion was skipped");
            require(!SubLevelTicketsSavedData.getOrLoad(server.overworld()).isDirty(), "Removed ticket was not saved");
        }
        result.put("restartChecksPass", true);
    }

    private void finish(MinecraftServer server) throws Exception {
        Files.writeString(Path.of(System.getenv("WE_BENCH_OUTPUT")),
                new GsonBuilder().setPrettyPrinting().create().toJson(result));
        ready = false;
        server.halt(false);
    }

    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
