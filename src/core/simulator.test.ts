import test from 'node:test';
import assert from 'node:assert/strict';
import { WarehouseGrid } from './grid';
import { runAStar, runDijkstra, runGreedyBestFirst } from './algorithms';
import { RobotSimulator } from './simulator';
import { runComparativeBenchmark, exportRecordsToCSV } from './benchmarking';

test('Grid environment enforces 4-directional movement and bounds', () => {
  const grid = new WarehouseGrid(10, 10);
  assert.equal(grid.rows, 10);
  assert.equal(grid.cols, 10);

  // Test corner neighbors (should be exactly 2 orthogonal neighbors)
  const cornerNeighbors = grid.getNeighbors(0, 0);
  assert.equal(cornerNeighbors.length, 2);
  const cornerDirs = cornerNeighbors.map((n) => n.direction).sort();
  assert.deepEqual(cornerDirs, ['DOWN', 'RIGHT']);

  // Test interior neighbors (should be exactly 4 orthogonal neighbors, no diagonals)
  const centerNeighbors = grid.getNeighbors(5, 5);
  assert.equal(centerNeighbors.length, 4);
  const centerDirs = centerNeighbors.map((n) => n.direction).sort();
  assert.deepEqual(centerDirs, ['DOWN', 'LEFT', 'RIGHT', 'UP']);

  // Test wall blocks neighbor
  grid.setCell(5, 6, 'WALL');
  const neighborsAfterWall = grid.getNeighbors(5, 5);
  assert.equal(neighborsAfterWall.length, 3);
  assert.equal(neighborsAfterWall.some((n) => n.coord.r === 5 && n.coord.c === 6), false);
});

test('Dijkstra and A* produce identical optimal cost on uniform grid', () => {
  const grid = new WarehouseGrid(10, 10);
  grid.start = { r: 1, c: 1 };
  grid.goal = { r: 8, c: 8 };

  // Place some obstacles requiring detour
  for (let r = 2; r <= 7; r++) {
    grid.setCell(r, 4, 'WALL');
  }

  const dijkstraRes = runDijkstra(grid);
  const astarRes = runAStar(grid);

  assert.equal(dijkstraRes.found, true);
  assert.equal(astarRes.found, true);
  assert.equal(dijkstraRes.totalCost, astarRes.totalCost);
  assert.equal(dijkstraRes.pathLength, astarRes.pathLength);

  // A* should expand fewer or equal nodes than Dijkstra when heuristic is informative
  assert.ok(
    astarRes.expandedNodes <= dijkstraRes.expandedNodes,
    `A* expanded (${astarRes.expandedNodes}) should be <= Dijkstra expanded (${dijkstraRes.expandedNodes})`
  );
});

test('Dijkstra and A* optimality on nonuniform weighted terrain', () => {
  const grid = new WarehouseGrid(8, 12);
  grid.start = { r: 4, c: 1 };
  grid.goal = { r: 4, c: 10 };

  // Center column has expensive terrain (cost 10)
  for (let r = 2; r <= 6; r++) {
    grid.setCell(r, 5, 'TERRAIN_VERY_HIGH');
  }
  // Low-cost detour at row 0 (cost 0.5)
  for (let c = 1; c <= 10; c++) {
    grid.setCell(0, c, 'TERRAIN_LOW');
  }

  const dijkstraRes = runDijkstra(grid);
  const astarRes = runAStar(grid);

  assert.equal(dijkstraRes.found, true);
  assert.equal(astarRes.found, true);
  // Both must find the optimal cost path
  assert.equal(dijkstraRes.totalCost, astarRes.totalCost);
});

test('Greedy Best-First Search is heuristic-driven and can be suboptimal on weighted terrain', () => {
  const grid = new WarehouseGrid(15, 25);
  grid.loadPreset('terrain_dilemma');

  const dijkstraRes = runDijkstra(grid);
  const greedyRes = runGreedyBestFirst(grid);

  assert.equal(dijkstraRes.found, true);
  assert.equal(greedyRes.found, true);

  // In terrain dilemma, Greedy rushes directly through the expensive swamp (higher total cost)
  // while Dijkstra/A* detours through the low-cost bypass (lower total cost)
  assert.ok(
    greedyRes.totalCost >= dijkstraRes.totalCost,
    `Greedy cost (${greedyRes.totalCost}) should be >= optimal Dijkstra cost (${dijkstraRes.totalCost})`
  );
});

test('Edge cases: start == goal, blocked start, blocked goal, unreachable', () => {
  const grid = new WarehouseGrid(6, 6);
  grid.start = { r: 2, c: 2 };
  grid.goal = { r: 2, c: 2 };

  const startEqualsGoal = runAStar(grid);
  assert.equal(startEqualsGoal.found, true);
  assert.equal(startEqualsGoal.pathLength, 0);
  assert.equal(startEqualsGoal.totalCost, 0);

  // Blocked goal
  grid.goal = { r: 4, c: 4 };
  grid.setCell(4, 4, 'WALL');
  const blockedGoal = runAStar(grid);
  assert.equal(blockedGoal.found, false);
  assert.ok(blockedGoal.errorMessage?.includes('Goal'));

  // Completely trapped goal
  grid.setCell(4, 4, 'EMPTY');
  grid.setCell(3, 4, 'WALL');
  grid.setCell(5, 4, 'WALL');
  grid.setCell(4, 3, 'WALL');
  grid.setCell(4, 5, 'WALL');
  const unreachable = runAStar(grid);
  assert.equal(unreachable.found, false);
  assert.ok(unreachable.errorMessage?.includes('No path'));
});

test('RobotSimulator dynamic obstacle insertion and replanning', () => {
  const grid = new WarehouseGrid(10, 10);
  grid.start = { r: 1, c: 1 };
  grid.goal = { r: 1, c: 8 };

  const sim = new RobotSimulator(grid, 'astar');
  const plan = sim.planMission();
  assert.equal(plan.success, true);
  assert.equal(sim.fullPlannedPath.length, 8);

  // Move robot 2 steps
  const step1 = sim.stepMovement();
  assert.equal(step1.status, 'moving');
  assert.equal(sim.state.position.r, 1);
  assert.equal(sim.state.position.c, 2);

  const step2 = sim.stepMovement();
  assert.equal(step2.status, 'moving');
  assert.equal(sim.state.position.r, 1);
  assert.equal(sim.state.position.c, 3);

  // Insert dynamic obstacle at (1, 5) ahead of the robot!
  const obstacleResult = sim.insertDynamicObstacle(1, 5);
  assert.equal(obstacleResult.success, true);
  assert.equal(obstacleResult.affectsPlannedRoute, true);
  assert.equal(obstacleResult.replanned, true);
  assert.equal(sim.state.replanCount, 1);

  // Robot position must have been preserved during replan
  assert.equal(sim.state.position.r, 1);
  assert.equal(sim.state.position.c, 3);

  // Robot must not enter the blocked cell (1, 5)
  while (sim.state.status === 'moving') {
    sim.stepMovement();
    assert.ok(
      !(sim.state.position.r === 1 && sim.state.position.c === 5),
      'Robot must never step on dynamic obstacle at (1, 5)'
    );
  }

  assert.equal(sim.state.status, 'completed');
  assert.equal(sim.state.position.r, 1);
  assert.equal(sim.state.position.c, 8);
});

test('Benchmarking snapshot isolation and CSV export', () => {
  const grid = new WarehouseGrid(10, 10);
  const bench = runComparativeBenchmark(grid, 'Test Grid');

  assert.ok(bench.astar.found);
  assert.ok(bench.dijkstra.found);
  assert.ok(bench.greedy.found);
  assert.equal(bench.records.length, 3);

  const csv = exportRecordsToCSV(bench.records);
  assert.ok(csv.includes('Scenario,Algorithm'));
  assert.ok(csv.includes('ASTAR'));
  assert.ok(csv.includes('DIJKSTRA'));
  assert.ok(csv.includes('GREEDY'));
});
