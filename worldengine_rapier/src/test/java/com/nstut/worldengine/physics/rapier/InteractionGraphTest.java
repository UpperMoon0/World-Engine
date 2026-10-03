package com.nstut.worldengine.physics.rapier;

import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import java.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InteractionGraphTest {
    @Test void movementAndRemovalMatchIndependentAllPairsOracle() {
        InteractionGraph graph = new InteractionGraph();
        Map<Integer, double[]> boxes = new HashMap<>();
        Random random = new Random(470101);
        for (int step = 0; step < 1200; step++) {
            int id = 1000 + random.nextInt(40);
            if (step % 11 == 0) { graph.remove(id); boxes.remove(id); }
            else {
                Set<Integer> previousSeeds = independentNeighbors(id, boxes);
                previousSeeds.add(id);
                double x = random.nextInt(600) - 300, y = random.nextInt(100), z = random.nextInt(300) - 150;
                double size = step % 47 == 0 ? 100_000 : 60;
                double[] b = {x, y, z, x + size, y + size, z + size};
                boxes.put(id, b);
                IntOpenHashSet affected = new IntOpenHashSet();
                graph.update(id, new InteractionGraph.Bounds(b[0], b[1], b[2], b[3], b[4], b[5]), affected);
                assertComponentsCovered(previousSeeds, affected, boxes, step);
            }
            if (step % 3 == 0 && !boxes.isEmpty()) {
                var unchanged = boxes.entrySet().iterator().next();
                double[] b = unchanged.getValue();
                Set<Integer> previousSeeds = independentNeighbors(unchanged.getKey(), boxes);
                previousSeeds.add(unchanged.getKey());
                IntOpenHashSet affected = new IntOpenHashSet();
                graph.update(unchanged.getKey(), new InteractionGraph.Bounds(b[0], b[1], b[2], b[3], b[4], b[5]), affected);
                assertComponentsCovered(previousSeeds, affected, boxes, step);
            }
            for (var left : boxes.entrySet()) {
                Set<Integer> expected = new HashSet<>();
                for (var right : boxes.entrySet()) {
                    double[] a = left.getValue(), b = right.getValue();
                    if (!left.getKey().equals(right.getKey()) && a[0] <= b[3] && a[3] >= b[0]
                            && a[1] <= b[4] && a[4] >= b[1] && a[2] <= b[5] && a[5] >= b[2]) expected.add(right.getKey());
                }
                assertEquals(expected, graph.neighbors(left.getKey()), "step " + step + ", id " + left.getKey());
            }
        }
    }

    private static Set<Integer> independentNeighbors(int id, Map<Integer, double[]> boxes) {
        Set<Integer> neighbors = new HashSet<>();
        double[] a = boxes.get(id);
        if (a == null) return neighbors;
        for (var entry : boxes.entrySet()) {
            double[] b = entry.getValue();
            if (entry.getKey() != id && a[0] <= b[3] && a[3] >= b[0]
                    && a[1] <= b[4] && a[4] >= b[1] && a[2] <= b[5] && a[5] >= b[2]) {
                neighbors.add(entry.getKey());
            }
        }
        return neighbors;
    }

    private static void assertComponentsCovered(Set<Integer> previousSeeds, Set<Integer> affected,
            Map<Integer, double[]> boxes, int step) {
        Set<Integer> reached = new HashSet<>(affected);
        List<Integer> pending = new ArrayList<>(affected);
        for (int head = 0; head < pending.size(); head++) {
            for (int neighbor : independentNeighbors(pending.get(head), boxes)) {
                if (reached.add(neighbor)) pending.add(neighbor);
            }
        }
        assertTrue(reached.containsAll(previousSeeds), "lost affected component at step " + step);
    }

    @Test void sameCellMovementStillUpdatesExactEdgesAndAffectedNeighbors() {
        InteractionGraph graph = new InteractionGraph();
        IntOpenHashSet affected = new IntOpenHashSet();
        graph.update(1, new InteractionGraph.Bounds(1, 1, 1, 3, 3, 3), affected);
        graph.update(2, new InteractionGraph.Bounds(3, 1, 1, 5, 3, 3), affected);
        assertEquals(Set.of(2), graph.neighbors(1)); // Inclusive touching boundary.
        affected.clear();
        graph.update(2, new InteractionGraph.Bounds(4, 1, 1, 6, 3, 3), affected);
        assertTrue(graph.neighbors(1).isEmpty());
        assertTrue(graph.neighbors(2).isEmpty());
        assertEquals(Set.of(1, 2), affected);
    }

    @Test void sharedPlaneMovementAndDepartureMatchExactOracle() {
        InteractionGraph graph = new InteractionGraph();
        Map<Integer, double[]> boxes = new HashMap<>();
        for (int step = 0; step < 1200; step++) {
            int id = step % 64;
            Set<Integer> previousSeeds = independentNeighbors(id, boxes);
            previousSeeds.add(id);
            double x = (id % 16) * 4, z = (id / 16) * 4;
            double y = step >= 400 && step < 800 && id == 0 ? 250 : Math.sin(step * .05);
            double[] b = {x - 8, y - 8, z - 8, x + 10, y + 9, z + 10};
            boxes.put(id, b);
            IntOpenHashSet affected = new IntOpenHashSet();
            graph.update(id, new InteractionGraph.Bounds(b[0], b[1], b[2], b[3], b[4], b[5]), affected);
            assertComponentsCovered(previousSeeds, affected, boxes, step);
            for (int present : boxes.keySet()) {
                assertEquals(independentNeighbors(present, boxes), graph.neighbors(present),
                        "plane transition at step " + step + ", id " + present);
            }
        }
        graph.clear();
        IntOpenHashSet affected = new IntOpenHashSet();
        graph.update(1, new InteractionGraph.Bounds(0, -1, 0, 2, 1, 2), affected);
        // The shared-plane shortcut must still update the expanded cell range.
        graph.update(1, new InteractionGraph.Bounds(0, -1, 0, 2, 300, 2), affected);
        graph.update(2, new InteractionGraph.Bounds(0, 280, 0, 2, 282, 2), affected);
        assertEquals(Set.of(2), graph.neighbors(1));
        assertEquals(Set.of(1), graph.neighbors(2));
        graph.remove(1);
        graph.remove(2);
        graph.update(3, new InteractionGraph.Bounds(0, 500, 0, 2, 502, 2), affected);
        graph.update(4, new InteractionGraph.Bounds(0, 500, 0, 2, 502, 2), affected);
        assertEquals(Set.of(4), graph.neighbors(3));
    }
}
