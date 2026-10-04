package com.nstut.worldengine.neoforge.gametest;

import com.nstut.worldengine.physics.WorldEngineBodyIndex;
import com.nstut.worldengine.physics.rapier.Rapier3D;
import com.nstut.worldengine.physics.rapier.RapierPhysicsPipeline;
import com.nstut.worldengine.physics.rapier.rope.RapierRopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopePhysicsObject;
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
    @GameTest(template = "physicstest.gravity", timeoutTicks = 100)
    public static void ropeAttachmentWaitsForTargetRegistration(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos block = helper.absolutePos(new BlockPos(2, 2, 2));
        level.setBlock(block.below(), Blocks.STONE.defaultBlockState(), 3);
        level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                new BoundingBox3i(block.getX(), block.getY(), block.getZ(), block.getX(), block.getY(), block.getZ()));
        RapierPhysicsPipeline pipeline = (RapierPhysicsPipeline) SubLevelPhysicsSystem.require(level).getPipeline();
        pipeline.remove(body);
        long auxiliary = Rapier3D.getSceneHandle(level);
        RapierRopeHandle rope = RapierRopeHandle.create(pipeline, auxiliary, 0.1,
                List.of(new Vector3d(block.getX(), block.getY(), block.getZ()),
                        new Vector3d(block.getX(), block.getY() + 1, block.getZ()),
                        new Vector3d(block.getX(), block.getY() + 2, block.getZ())));
        rope.setAttachment(RopeHandle.AttachmentPoint.START, new Vector3d(), body);
        if (rope.sceneHandle() != auxiliary) helper.fail("Unregistered target was materialized prematurely");
        pipeline.add(body, body.logicalPose());
        helper.startSequence().thenIdle(2).thenExecute(() -> {
            if (rope.sceneHandle() == auxiliary || rope.sceneHandle() != pipeline.prepareConstraintScene(body, null)) {
                helper.fail("Deferred attachment did not move into the newly registered target's scene");
            }
            rope.remove();
        }).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 240)
    public static void ropeAttachmentSurvivesMissingAndDormantBodies(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos support = helper.absolutePos(new BlockPos(2, 1, 2));
        BlockPos block = support.above();
        level.setBlock(support, Blocks.STONE.defaultBlockState(), 3);
        level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
        ServerSubLevel body = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                new BoundingBox3i(block.getX(), block.getY(), block.getZ(), block.getX(), block.getY(), block.getZ()));
        RapierPhysicsPipeline pipeline = (RapierPhysicsPipeline) SubLevelPhysicsSystem.require(level).getPipeline();
        long scene = Rapier3D.getSceneHandle(level);
        List<Vector3d> points = List.of(new Vector3d(block.getX(), block.getY() + 2, block.getZ()),
                new Vector3d(block.getX(), block.getY() + 1, block.getZ()),
                new Vector3d(block.getX(), block.getY(), block.getZ()));
        RapierRopeHandle probe = RapierRopeHandle.create(pipeline, scene, 0.1, points);
        // Call the actual JNI boundary with an absent target: old main aborts here.
        Rapier3D.setRopeAttachment(scene, probe.handle(), Integer.MAX_VALUE, 0, 0, 0, false);
        Rapier3D.setRopeAttachment(scene, Long.MAX_VALUE, Integer.MAX_VALUE, 0, 0, 0, true);
        probe.remove();
        RopePhysicsObject rope = new RopePhysicsObject(points, 0.1);
        rope.onAddition(SubLevelPhysicsSystem.require(level));
        rope.setAttachment(RopeHandle.AttachmentPoint.END, points.getLast(), null);
        helper.startSequence().thenIdle(160).thenExecute(() -> {
            if (((WorldEnginePhysicsSystem) SubLevelPhysicsSystem.require(level)).worldengine$activeBodies().contains(body)) {
                helper.fail("Rope fixture body did not become dormant");
            }
            long localScene = pipeline.prepareConstraintScene(body, null);
            rope.setAttachment(RopeHandle.AttachmentPoint.START, new Vector3d(), body);
            if (pipeline.prepareConstraintScene(body, null) != localScene || localScene == scene) {
                helper.fail("Rope attachment moved the target out of its local scene");
            }
        }).thenExecuteFor(20, () -> {
            rope.updatePose();
            for (Vector3d point : rope.getPoints()) {
                if (!Double.isFinite(point.x) || !Double.isFinite(point.y) || !Double.isFinite(point.z)) {
                    helper.fail("Rope pose became non-finite");
                }
            }
        }).thenExecute(rope::onRemoved).thenSucceed();
    }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 100)
    public static void ropeAttachmentKeepsExistingConstraints(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        ServerSubLevel[] bodies = new ServerSubLevel[2];
        for (int i = 0; i < 2; i++) {
            BlockPos block = helper.absolutePos(new BlockPos(2 + i * 2, 2, 2));
            level.setBlock(block.below(), Blocks.STONE.defaultBlockState(), 3);
            level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
            bodies[i] = SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                    new BoundingBox3i(block.getX(), block.getY(), block.getZ(), block.getX(), block.getY(), block.getZ()));
        }
        RapierPhysicsPipeline pipeline = (RapierPhysicsPipeline) SubLevelPhysicsSystem.require(level).getPipeline();
        helper.startSequence().thenIdle(2).thenExecute(() -> {
            long originalScene = pipeline.prepareConstraintScene(bodies[0], bodies[1]);
            long constraint = Rapier3D.addFixedConstraint(originalScene, Rapier3D.getID(bodies[0]), Rapier3D.getID(bodies[1]),
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
            long ropeScene = Rapier3D.getSceneHandle(level);
            if (originalScene == ropeScene) helper.fail("Fixture did not create a separate constrained scene");
            RopePhysicsObject rope = new RopePhysicsObject(
                    List.of(new Vector3d(0, 3, 0), new Vector3d(0, 2, 0), new Vector3d(0, 1, 0)), 0.1);
            rope.onAddition(SubLevelPhysicsSystem.require(level));
            rope.setAttachment(RopeHandle.AttachmentPoint.START, new Vector3d(), bodies[0]);
            if (pipeline.prepareConstraintScene(bodies[0], bodies[1]) != originalScene
                    || !Rapier3D.isConstraintValid(originalScene, constraint)) {
                helper.fail("Rope attachment lost an existing body constraint");
            }
            // Exercise reversed ordering with the auxiliary scene as body B.
            if (pipeline.prepareConstraintScene(bodies[1], bodies[0]) != originalScene) {
                helper.fail("Constraint preparation moved a rope-pinned body out of its scene");
            }
            rope.onRemoved();
            Rapier3D.removeConstraint(originalScene, constraint);
        }).thenSucceed();
    }

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
    @GameTest(template = "physicstest.gravity", timeoutTicks = 200)
    public static void interactingBodiesKeepSupportAcrossRegionBoundary(GameTestHelper helper) {
        ServerLevel level = helper.getLevel();
        BlockPos origin = helper.absolutePos(BlockPos.ZERO);
        // Deliberately straddle the nearest 4096-block region boundary. Keep
        // this fixture away from the ordinary structure grid and its bodies.
        int boundary = (int) Math.floor((origin.getX() + 2048.0) / 4096.0) * 4096 + 2048;
        int z = origin.getZ() + 128;
        List<ServerSubLevel> bodies = new java.util.ArrayList<>();
        for (int x : new int[]{boundary - 2, boundary + 2}) {
            level.setChunkForced(x >> 4, z >> 4, true);
            BlockPos floor = new BlockPos(x, origin.getY() + 1, z);
            for (int dx = -1; dx <= 1; dx++) for (int dz = -1; dz <= 1; dz++) {
                for (int dy = 0; dy <= 3; dy++) {
                    level.setBlock(floor.offset(dx, dy, dz), dy == 0
                            ? Blocks.STONE.defaultBlockState() : Blocks.AIR.defaultBlockState(), 3);
                }
            }
            BlockPos block = floor.above();
            level.setBlock(block, Blocks.DIAMOND_BLOCK.defaultBlockState(), 3);
            bodies.add(SubLevelAssemblyHelper.assembleBlocks(level, block, List.of(block),
                    new BoundingBox3i(block.getX(), block.getY(), block.getZ(), block.getX(), block.getY(), block.getZ())));
        }
        helper.startSequence().thenIdle(160).thenExecute(() -> {
            for (int x : new int[]{boundary - 2, boundary + 2}) level.setChunkForced(x >> 4, z >> 4, false);
            for (ServerSubLevel body : bodies) {
                double y = body.logicalPose().position().y();
                if (body.isRemoved() || !Double.isFinite(y) || Math.abs(y - (origin.getY() + 2.5)) > 0.15) {
                    helper.fail("Body lost terrain support while merging boundary regions: y=" + y
                            + ", expected=" + (origin.getY() + 2.5) + ", removed=" + body.isRemoved());
                }
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
            boolean found = false;
            for (SubLevel result : system.queryIntersecting(body.boundingBox())) {
                if (result == body) found = true;
            }
            if (!found) helper.fail("Exact query missed the live ballistic body after falling: y="
                    + body.logicalPose().position().y() + ", bounds=" + body.boundingBox());
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
