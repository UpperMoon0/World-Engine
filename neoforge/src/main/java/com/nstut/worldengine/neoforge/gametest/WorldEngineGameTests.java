package com.nstut.worldengine.neoforge.gametest;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.SubLevelAssemblyHelper;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.GameTestHolder;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;

@GameTestHolder(Sable.MOD_ID)
public final class WorldEngineGameTests {
    private WorldEngineGameTests() { }

    @PrefixGameTestTemplate(false)
    @GameTest(template = "physicstest.gravity", timeoutTicks = 80)
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
                .thenExecuteFor(40, () -> {
                    if (subLevel.isRemoved()) helper.fail("Assembled sublevel was removed");
                    if (subLevel.logicalPose().position().y() < startingY - 1.0) {
                        helper.fail("Assembled sublevel fell through its terrain support");
                    }
                })
                .thenSucceed();
    }
}
