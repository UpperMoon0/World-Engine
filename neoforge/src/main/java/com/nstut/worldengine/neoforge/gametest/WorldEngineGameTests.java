package com.nstut.worldengine.neoforge.gametest;

import com.nstut.worldengine.physics.WorldEngineBodyIndex;
import com.nstut.worldengine.api.WorldEngineTerrainBodies;
import com.nstut.worldengine.api.WorldEnginePhysicsSystem;
import com.nstut.worldengine.api.WorldEngineSolverConfiguration;
import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.SubLevelAssemblyHelper;
import dev.ryanhcode.sable.companion.math.BoundingBox3d;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.physics.config.dimension_physics.DimensionPhysicsData;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.GameTestHolder;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;
import org.joml.Vector3d;

@GameTestHolder(Sable.MOD_ID)
public final class WorldEngineGameTests {
    private WorldEngineGameTests() { }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 200)
    public static void assembledBodyKeepsTerrainSupport(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos support = helper.absolutePos(new BlockPos(2, 1, 2));
        BlockPos assembledBlock = support.above();
        level.setBlock(support, Blocks.STONE.defaultBlockState(), 3);
        level.setBlock(assembledBlock, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);

        BoundingBox3i bounds = new BoundingBox3i(
                assembledBlock.getX(), assembledBlock.getY(), assembledBlock.getZ(),
                assembledBlock.getX(), assembledBlock.getY(), assembledBlock.getZ());
        ServerSubLevel subLevel = SubLevelAssemblyHelper.assembleBlocks(
                level, assembledBlock, List.of(assembledBlock), bounds);
        double startingY = subLevel.logicalPose().position().y();

        helper.startSequence()
                .thenExecuteFor(160, () -> {
                    // Keep checking after native sleep and the 20-tick terrain ticket expiry.
                    if (subLevel.isRemoved()) helper.fail("Assembled sublevel was removed");
                    if (subLevel.logicalPose().position().y() < startingY - 1.0) {
                        helper.fail("Assembled sublevel fell through its terrain support: initialY="
                                + startingY + ", final=" + subLevel.logicalPose().position()
                                + ", support=" + level.getBlockState(support));
                    }
                })
                .thenExecute(() -> {
                    SubLevelPhysicsSystem system = SubLevelPhysicsSystem.require(level);
                    if (!(system.getPipeline() instanceof WorldEngineSolverConfiguration configuration)) {
                        helper.fail("Solver configuration diagnostics unavailable");
                        return;
                    }
                    var applied = configuration.worldengine$appliedSolverSettings();
                    if (applied.size() < 2) helper.fail("Fixture did not create a late body region");
                    var expected = WorldEngineSolverConfiguration.Settings.from(system.getConfig());
                    if (applied.values().stream().anyMatch(settings -> !settings.equals(expected))) {
                        helper.fail("Late physics region did not inherit solver settings");
                    }
                })
                .thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 240)
    public static void sleepingBodyFallsAfterSupportRemoval(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos support = helper.absolutePos(new BlockPos(2, 1, 2));
        BlockPos block = support.above();
        level.setBlock(support, Blocks.STONE.defaultBlockState(), 3);
        level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                new BoundingBox3i(block.getX(), block.getY(), block.getZ(),
                        block.getX(), block.getY(), block.getZ()));
        double[] supportedY = new double[1];
        helper.startSequence().thenIdle(160).thenExecute(() -> {
            supportedY[0] = body.logicalPose().position().y();
            if (body.isRemoved() || Math.abs(supportedY[0] - (block.getY() + 0.5)) > 0.15) {
                helper.fail("Body did not settle on the support before removal");
            }
            if (((WorldEnginePhysicsSystem) SubLevelPhysicsSystem.require(level)).worldengine$activeBodies().contains(body)) {
                helper.fail("Support-removal fixture was still active before the terrain change");
            }
            // Use the production block-change path, without an explicit test wake-up.
            level.setBlock(support, Blocks.AIR.defaultBlockState(), 3);
        }).thenIdle(40).thenExecute(() -> {
            double y = body.logicalPose().position().y();
            if (body.isRemoved() || !Double.isFinite(y) || y >= supportedY[0] - 0.5) {
                helper.fail("Supported body did not wake and fall after terrain removal");
            }
        }).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 300)
    public static void cuboidWithNewCenterHoleFallsPastItsFormerSupport(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        for (int x = 0; x <= 4; x++) for (int z = 0; z <= 4; z++) {
            for (int y = 1; y <= 4; y++) {
                level.setBlock(helper.absolutePos(new BlockPos(x, y, z)), Blocks.AIR.defaultBlockState(), 3);
            }
        }
        BlockPos pillar = helper.absolutePos(new BlockPos(2, 1, 2));
        level.setBlock(pillar, Blocks.STONE.defaultBlockState(), 3);
        List<BlockPos> blocks = new java.util.ArrayList<>();
        for (int x = 1; x <= 3; x++) for (int z = 1; z <= 3; z++) {
            BlockPos block = helper.absolutePos(new BlockPos(x, 2, z));
            level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
            blocks.add(block);
        }
        BlockPos min = blocks.getFirst(), max = blocks.getLast();
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, min, blocks,
                new BoundingBox3i(min.getX(), min.getY(), min.getZ(), max.getX(), max.getY(), max.getZ()));
        double[] supportedY = new double[1];
        helper.startSequence().thenIdle(160).thenExecute(() -> {
            supportedY[0] = body.logicalPose().position().y();
            if (body.isRemoved() || Math.abs(supportedY[0] - (pillar.getY() + 1.5)) > 0.15) {
                helper.fail("Full cuboid did not settle on its center pillar");
            }
            // Mass coordinates identify the middle voxel in the assembled plot,
            // independent of the plot allocator's padding and remote location.
            var center = body.getMassTracker().getCenterOfMass();
            BlockPos centerBlock = new BlockPos((int) Math.floor(center.x()),
                    (int) Math.floor(center.y()), (int) Math.floor(center.z()));
            if (!level.getBlockState(centerBlock).is(Blocks.DIAMOND_BLOCK)) {
                helper.fail("Center-hole fixture did not resolve the assembled center voxel");
            }
            level.setBlock(centerBlock, Blocks.AIR.defaultBlockState(), 3);
        }).thenIdle(60).thenExecute(() -> {
            double y = body.logicalPose().position().y();
            if (body.isRemoved() || !Double.isFinite(y) || y >= supportedY[0] - 0.5) {
                helper.fail("Body retained collision in its edited center hole");
            }
        }).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 240)
    public static void terrainRectangleRebuildsAfterSupportRemoval(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        for (int x = 0; x <= 4; x++) for (int z = 0; z <= 4; z++) for (int y = 1; y <= 4; y++) {
            level.setBlock(helper.absolutePos(new BlockPos(x, y, z)), Blocks.AIR.defaultBlockState(), 3);
        }
        List<BlockPos> floor = new java.util.ArrayList<>();
        for (int x = 1; x <= 3; x++) for (int z = 1; z <= 3; z++) {
            BlockPos pos = helper.absolutePos(new BlockPos(x, 1, z));
            level.setBlock(pos, Blocks.STONE.defaultBlockState(), 3);
            floor.add(pos);
        }
        BlockPos block = helper.absolutePos(new BlockPos(2, 2, 2));
        level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                new BoundingBox3i(block.getX(), block.getY(), block.getZ(), block.getX(), block.getY(), block.getZ()));
        double[] supportedY = new double[1];
        helper.startSequence().thenIdle(160).thenExecute(() -> {
            supportedY[0] = body.logicalPose().position().y();
            if (body.isRemoved() || Math.abs(supportedY[0] - (block.getY() + 0.5)) > 0.15) {
                helper.fail("Body did not settle on the terrain rectangle");
            }
            if (((WorldEnginePhysicsSystem) SubLevelPhysicsSystem.require(level)).worldengine$activeBodies().contains(body)) {
                helper.fail("Rectangle support-removal fixture was still active");
            }
            for (BlockPos pos : floor) level.setBlock(pos, Blocks.AIR.defaultBlockState(), 3);
        }).thenIdle(40).thenExecute(() -> {
            double y = body.logicalPose().position().y();
            if (body.isRemoved() || !Double.isFinite(y) || y >= supportedY[0] - 0.5) {
                helper.fail("Body retained collision with a removed terrain rectangle");
            }
        }).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 180)
    public static void partialTerrainRetainsHalfHeightSupport(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos slab = helper.absolutePos(new BlockPos(2, 1, 2));
        level.setBlock(slab, Blocks.STONE_SLAB.defaultBlockState(), 3);
        BlockPos block = slab.above(2);
        level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                new BoundingBox3i(block.getX(), block.getY(), block.getZ(), block.getX(), block.getY(), block.getZ()));
        helper.startSequence().thenIdle(120).thenExecute(() -> {
            double y = body.logicalPose().position().y();
            if (body.isRemoved() || !Double.isFinite(y) || Math.abs(y - (slab.getY() + 1.0)) > 0.1) {
                helper.fail("Partial terrain was treated as a full cube or lost its collision shape");
            }
        }).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 80)
    public static void ballisticGravityUsesServerTickTime(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos block = helper.absolutePos(new BlockPos(2, 2, 2));
        level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                new BoundingBox3i(block.getX(), block.getY(), block.getZ(),
                        block.getX(), block.getY(), block.getZ()));
        SubLevelPhysicsSystem system = SubLevelPhysicsSystem.require(level);
        Vector3d position = new Vector3d(body.logicalPose().position());
        position.y = 10_000.0; // Outside terrain coverage: exercise abstract ballistic integration.
        system.getPhysicsHandle(body).teleport(position, body.logicalPose().orientation());
        body.updateBoundingBox();
        double expectedFall = -0.5 * DimensionPhysicsData.getGravity(level).y() * 2.5 * 2.5;
        helper.startSequence().thenIdle(50).thenExecute(() -> {
            if (body.isRemoved()) helper.fail("Ballistic body was removed");
            if (!(system.getPipeline() instanceof WorldEngineTerrainBodies terrain)
                    || terrain.worldengine$ticketBodies(List.of()).contains(body)) {
                helper.fail("Fixture did not exercise abstract ballistic integration");
            }
            double fall = position.y - body.logicalPose().position().y();
            // Abstract poses update at most once per second. Allow that scheduler
            // lag while rejecting the doubled elapsed time from two solver substeps.
            if (!Double.isFinite(fall) || fall < expectedFall * 0.5 || fall > expectedFall * 1.2) {
                helper.fail("Ballistic elapsed time differs from server time: fall=" + fall);
            }
        }).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 20)
    public static void worldScaleQueryUsesIndexedBodies(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos assembledBlock = helper.absolutePos(new BlockPos(2, 2, 2));
        level.setBlock(assembledBlock, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);

        BoundingBox3i bodyBounds = new BoundingBox3i(
                assembledBlock.getX(), assembledBlock.getY(), assembledBlock.getZ(),
                assembledBlock.getX(), assembledBlock.getY(), assembledBlock.getZ());
        ServerSubLevel subLevel = SubLevelAssemblyHelper.assembleBlocks(
                level, assembledBlock, List.of(assembledBlock), bodyBounds);
        WorldEngineBodyIndex index = new WorldEngineBodyIndex();
        index.update(subLevel);

        BoundingBox3d worldBounds = new BoundingBox3d(
                -30_000_000, -10_000, -30_000_000,
                30_000_000, 10_000, 30_000_000);
        for (SubLevel result : index.query(worldBounds)) {
            if (result == subLevel) {
                helper.succeed();
                return;
            }
        }
        helper.fail("World-scale query did not return the indexed sublevel");
    }
}
