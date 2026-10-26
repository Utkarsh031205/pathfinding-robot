import {
  AlgorithmResult,
  AlgorithmType,
  Coordinate,
  SearchEvent,
} from '../types/simulator';
import { WarehouseGrid } from './grid';
import { PriorityQueue } from './priorityQueue';
import { manhattanDistance, scaledManhattanDistance } from './heuristics';

/**
 * Key serializer for coordinate tracking in Maps and Sets.
 */
export function coordKey(c: Coordinate): string {
  return `${c.r},${c.c}`;
}

export function parseCoordKey(key: string): Coordinate {
  const [r, c] = key.split(',').map(Number);
  return { r, c };
}

/**
 * Reconstructs path from parent map starting at goal back to start.
 */
function reconstructPath(
  parentMap: Map<string, Coordinate>,
  start: Coordinate,
  goal: Coordinate
): Coordinate[] {
  const path: Coordinate[] = [];
  let currKey: string | undefined = coordKey(goal);
  const startKey = coordKey(start);

  while (currKey) {
    const coord = parseCoordKey(currKey);
    path.push(coord);
    if (currKey === startKey) break;
    const parent = parentMap.get(currKey);
    currKey = parent ? coordKey(parent) : undefined;
  }

  return path.reverse();
}

/**
 * Computes exact cumulative traversal cost along an ordered path of coordinates.
 * Cost model: entering cell v adds getCellCost(v.r, v.c). Start cell entry cost is 0.
 */
export function computePathCost(grid: WarehouseGrid, path: Coordinate[]): number {
  if (path.length <= 1) return 0;
  let total = 0;
  for (let i = 1; i < path.length; i++) {
    total += grid.getCellCost(path[i].r, path[i].c);
  }
  return Number(total.toFixed(4));
}

/**
 * Execute Dijkstra's Algorithm:
 * Optimal uniform-cost exploration using priority queue on g(n).
 */
export function runDijkstra(
  grid: WarehouseGrid,
  recordEvents: boolean = true
): AlgorithmResult {
  const startTime = performance.now();
  const searchEvents: SearchEvent[] = [];
  let eventStep = 0;

  const start = grid.start;
  const goal = grid.goal;
  const startKey = coordKey(start);
  const goalKey = coordKey(goal);

  // Validation: start and goal identical
  if (start.r === goal.r && start.c === goal.c) {
    const elapsed = performance.now() - startTime;
    return {
      algorithm: 'dijkstra',
      found: true,
      path: [{ ...start }],
      expandedNodes: 1,
      discoveredNodes: 1,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number(elapsed.toFixed(3)),
      searchEvents: [
        {
          step: 1,
          type: 'GOAL_REACHED',
          coord: { ...start },
          gScore: 0,
          fScore: 0,
          hScore: 0,
        },
      ],
      exploredCoords: [{ ...start }],
      frontierCoords: [],
    };
  }

  // Validation: start or goal blocked
  if (!grid.isPassable(start.r, start.c)) {
    return {
      algorithm: 'dijkstra',
      found: false,
      path: [],
      expandedNodes: 0,
      discoveredNodes: 0,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number((performance.now() - startTime).toFixed(3)),
      searchEvents: [],
      errorMessage: 'Start position is blocked or impassable.',
      exploredCoords: [],
      frontierCoords: [],
    };
  }

  if (!grid.isPassable(goal.r, goal.c)) {
    return {
      algorithm: 'dijkstra',
      found: false,
      path: [],
      expandedNodes: 0,
      discoveredNodes: 0,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number((performance.now() - startTime).toFixed(3)),
      searchEvents: [],
      errorMessage: 'Goal position is blocked or impassable.',
      exploredCoords: [],
      frontierCoords: [],
    };
  }

  const pq = new PriorityQueue<{ coord: Coordinate; g: number }>();
  const gScores = new Map<string, number>();
  const parentMap = new Map<string, Coordinate>();
  const closedSet = new Set<string>();
  const discoveredSet = new Set<string>();

  gScores.set(startKey, 0);
  pq.push({ coord: start, g: 0 }, 0);
  discoveredSet.add(startKey);

  if (recordEvents) {
    searchEvents.push({
      step: ++eventStep,
      type: 'DISCOVER',
      coord: { ...start },
      gScore: 0,
      fScore: 0,
      hScore: 0,
    });
  }

  let expandedCount = 0;
  let generatedCount = 0;
  let goalReached = false;

  while (!pq.isEmpty()) {
    const current = pq.pop()!;
    const currKey = coordKey(current.coord);

    // Stale entry check: if we already closed this node or found a strictly better path
    if (closedSet.has(currKey)) continue;
    const bestKnownG = gScores.get(currKey) ?? Infinity;
    if (current.g > bestKnownG) continue;

    // Settle node (expand)
    closedSet.add(currKey);
    expandedCount++;

    if (recordEvents) {
      searchEvents.push({
        step: ++eventStep,
        type: 'EXPAND',
        coord: { ...current.coord },
        gScore: current.g,
        fScore: current.g,
        hScore: 0,
      });
    }

    // Goal test: terminate when goal is popped from priority queue
    if (currKey === goalKey) {
      goalReached = true;
      if (recordEvents) {
        searchEvents.push({
          step: ++eventStep,
          type: 'GOAL_REACHED',
          coord: { ...goal },
          gScore: current.g,
          fScore: current.g,
          hScore: 0,
        });
      }
      break;
    }

    const neighbors = grid.getNeighbors(current.coord.r, current.coord.c);
    for (const neighbor of neighbors) {
      generatedCount++;
      const nKey = coordKey(neighbor.coord);
      if (closedSet.has(nKey)) continue;

      const tentativeG = current.g + neighbor.cost;
      const prevG = gScores.get(nKey) ?? Infinity;

      if (tentativeG < prevG) {
        gScores.set(nKey, tentativeG);
        parentMap.set(nKey, current.coord);
        pq.push({ coord: neighbor.coord, g: tentativeG }, tentativeG);

        const isNewDiscovery = !discoveredSet.has(nKey);
        discoveredSet.add(nKey);

        if (recordEvents) {
          searchEvents.push({
            step: ++eventStep,
            type: isNewDiscovery ? 'DISCOVER' : 'UPDATE_G',
            coord: { ...neighbor.coord },
            gScore: tentativeG,
            fScore: tentativeG,
            hScore: 0,
          });
        }
      }
    }
  }

  const elapsed = performance.now() - startTime;

  if (!goalReached) {
    return {
      algorithm: 'dijkstra',
      found: false,
      path: [],
      expandedNodes: expandedCount,
      discoveredNodes: discoveredSet.size,
      generatedNodes: generatedCount,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number(elapsed.toFixed(3)),
      searchEvents,
      errorMessage: 'No path exists to the goal.',
      exploredCoords: Array.from(closedSet).map(parseCoordKey),
      frontierCoords: Array.from(discoveredSet)
        .filter((k) => !closedSet.has(k))
        .map(parseCoordKey),
    };
  }

  const path = reconstructPath(parentMap, start, goal);
  const totalCost = computePathCost(grid, path);

  return {
    algorithm: 'dijkstra',
    found: true,
    path,
    expandedNodes: expandedCount,
    discoveredNodes: discoveredSet.size,
    generatedNodes: generatedCount,
    totalCost,
    pathLength: path.length > 0 ? path.length - 1 : 0,
    executionTimeMs: Number(elapsed.toFixed(3)),
    searchEvents,
    exploredCoords: Array.from(closedSet).map(parseCoordKey),
    frontierCoords: Array.from(discoveredSet)
      .filter((k) => !closedSet.has(k))
      .map(parseCoordKey),
  };
}

/**
 * Execute A* Search:
 * f(n) = g(n) + h(n), where h(n) = scaledManhattanDistance(n, goal, minMovementCost).
 * Admissible and optimal for nonuniform positive costs.
 */
export function runAStar(
  grid: WarehouseGrid,
  recordEvents: boolean = true
): AlgorithmResult {
  const startTime = performance.now();
  const searchEvents: SearchEvent[] = [];
  let eventStep = 0;

  const start = grid.start;
  const goal = grid.goal;
  const startKey = coordKey(start);
  const goalKey = coordKey(goal);

  // Validation: start and goal identical
  if (start.r === goal.r && start.c === goal.c) {
    const elapsed = performance.now() - startTime;
    return {
      algorithm: 'astar',
      found: true,
      path: [{ ...start }],
      expandedNodes: 1,
      discoveredNodes: 1,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number(elapsed.toFixed(3)),
      searchEvents: [
        {
          step: 1,
          type: 'GOAL_REACHED',
          coord: { ...start },
          gScore: 0,
          fScore: 0,
          hScore: 0,
        },
      ],
      exploredCoords: [{ ...start }],
      frontierCoords: [],
    };
  }

  if (!grid.isPassable(start.r, start.c)) {
    return {
      algorithm: 'astar',
      found: false,
      path: [],
      expandedNodes: 0,
      discoveredNodes: 0,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number((performance.now() - startTime).toFixed(3)),
      searchEvents: [],
      errorMessage: 'Start position is blocked or impassable.',
      exploredCoords: [],
      frontierCoords: [],
    };
  }

  if (!grid.isPassable(goal.r, goal.c)) {
    return {
      algorithm: 'astar',
      found: false,
      path: [],
      expandedNodes: 0,
      discoveredNodes: 0,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number((performance.now() - startTime).toFixed(3)),
      searchEvents: [],
      errorMessage: 'Goal position is blocked or impassable.',
      exploredCoords: [],
      frontierCoords: [],
    };
  }

  const minCost = grid.getMinMovementCost();
  const pq = new PriorityQueue<{ coord: Coordinate; g: number; f: number }>();
  const gScores = new Map<string, number>();
  const parentMap = new Map<string, Coordinate>();
  const closedSet = new Set<string>();
  const discoveredSet = new Set<string>();

  const startH = scaledManhattanDistance(start, goal, minCost);
  gScores.set(startKey, 0);
  pq.push({ coord: start, g: 0, f: startH }, startH, startH);
  discoveredSet.add(startKey);

  if (recordEvents) {
    searchEvents.push({
      step: ++eventStep,
      type: 'DISCOVER',
      coord: { ...start },
      gScore: 0,
      fScore: startH,
      hScore: startH,
    });
  }

  let expandedCount = 0;
  let generatedCount = 0;
  let goalReached = false;

  while (!pq.isEmpty()) {
    const current = pq.pop()!;
    const currKey = coordKey(current.coord);

    if (closedSet.has(currKey)) continue;
    const bestKnownG = gScores.get(currKey) ?? Infinity;
    if (current.g > bestKnownG) continue;

    closedSet.add(currKey);
    expandedCount++;

    const currH = scaledManhattanDistance(current.coord, goal, minCost);

    if (recordEvents) {
      searchEvents.push({
        step: ++eventStep,
        type: 'EXPAND',
        coord: { ...current.coord },
        gScore: current.g,
        fScore: current.f,
        hScore: currH,
      });
    }

    // Goal test upon popping from priority queue
    if (currKey === goalKey) {
      goalReached = true;
      if (recordEvents) {
        searchEvents.push({
          step: ++eventStep,
          type: 'GOAL_REACHED',
          coord: { ...goal },
          gScore: current.g,
          fScore: current.f,
          hScore: 0,
        });
      }
      break;
    }

    const neighbors = grid.getNeighbors(current.coord.r, current.coord.c);
    for (const neighbor of neighbors) {
      generatedCount++;
      const nKey = coordKey(neighbor.coord);
      if (closedSet.has(nKey)) continue;

      const tentativeG = current.g + neighbor.cost;
      const prevG = gScores.get(nKey) ?? Infinity;

      if (tentativeG < prevG) {
        gScores.set(nKey, tentativeG);
        parentMap.set(nKey, current.coord);

        const h = scaledManhattanDistance(neighbor.coord, goal, minCost);
        const f = tentativeG + h;

        // Tie-breaker: prefer smaller h (closer to goal) for identical f
        pq.push({ coord: neighbor.coord, g: tentativeG, f }, f, h);

        const isNewDiscovery = !discoveredSet.has(nKey);
        discoveredSet.add(nKey);

        if (recordEvents) {
          searchEvents.push({
            step: ++eventStep,
            type: isNewDiscovery ? 'DISCOVER' : 'UPDATE_G',
            coord: { ...neighbor.coord },
            gScore: tentativeG,
            fScore: f,
            hScore: h,
          });
        }
      }
    }
  }

  const elapsed = performance.now() - startTime;

  if (!goalReached) {
    return {
      algorithm: 'astar',
      found: false,
      path: [],
      expandedNodes: expandedCount,
      discoveredNodes: discoveredSet.size,
      generatedNodes: generatedCount,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number(elapsed.toFixed(3)),
      searchEvents,
      errorMessage: 'No path exists to the goal.',
      exploredCoords: Array.from(closedSet).map(parseCoordKey),
      frontierCoords: Array.from(discoveredSet)
        .filter((k) => !closedSet.has(k))
        .map(parseCoordKey),
    };
  }

  const path = reconstructPath(parentMap, start, goal);
  const totalCost = computePathCost(grid, path);

  return {
    algorithm: 'astar',
    found: true,
    path,
    expandedNodes: expandedCount,
    discoveredNodes: discoveredSet.size,
    generatedNodes: generatedCount,
    totalCost,
    pathLength: path.length > 0 ? path.length - 1 : 0,
    executionTimeMs: Number(elapsed.toFixed(3)),
    searchEvents,
    exploredCoords: Array.from(closedSet).map(parseCoordKey),
    frontierCoords: Array.from(discoveredSet)
      .filter((k) => !closedSet.has(k))
      .map(parseCoordKey),
  };
}

/**
 * Execute Greedy Best-First Search:
 * priority(n) = h(n) = Manhattan(n, goal).
 * Focuses purely on estimated distance to goal without considering path cost.
 * Reports actual true weighted cost of resulting route (often suboptimal).
 */
export function runGreedyBestFirst(
  grid: WarehouseGrid,
  recordEvents: boolean = true
): AlgorithmResult {
  const startTime = performance.now();
  const searchEvents: SearchEvent[] = [];
  let eventStep = 0;

  const start = grid.start;
  const goal = grid.goal;
  const startKey = coordKey(start);
  const goalKey = coordKey(goal);

  if (start.r === goal.r && start.c === goal.c) {
    const elapsed = performance.now() - startTime;
    return {
      algorithm: 'greedy',
      found: true,
      path: [{ ...start }],
      expandedNodes: 1,
      discoveredNodes: 1,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number(elapsed.toFixed(3)),
      searchEvents: [
        {
          step: 1,
          type: 'GOAL_REACHED',
          coord: { ...start },
          gScore: 0,
          fScore: 0,
          hScore: 0,
        },
      ],
      exploredCoords: [{ ...start }],
      frontierCoords: [],
    };
  }

  if (!grid.isPassable(start.r, start.c)) {
    return {
      algorithm: 'greedy',
      found: false,
      path: [],
      expandedNodes: 0,
      discoveredNodes: 0,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number((performance.now() - startTime).toFixed(3)),
      searchEvents: [],
      errorMessage: 'Start position is blocked or impassable.',
      exploredCoords: [],
      frontierCoords: [],
    };
  }

  if (!grid.isPassable(goal.r, goal.c)) {
    return {
      algorithm: 'greedy',
      found: false,
      path: [],
      expandedNodes: 0,
      discoveredNodes: 0,
      generatedNodes: 0,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number((performance.now() - startTime).toFixed(3)),
      searchEvents: [],
      errorMessage: 'Goal position is blocked or impassable.',
      exploredCoords: [],
      frontierCoords: [],
    };
  }

  const pq = new PriorityQueue<{ coord: Coordinate; h: number }>();
  const parentMap = new Map<string, Coordinate>();
  const closedSet = new Set<string>();
  const discoveredSet = new Set<string>();

  const startH = manhattanDistance(start, goal);
  pq.push({ coord: start, h: startH }, startH);
  discoveredSet.add(startKey);

  if (recordEvents) {
    searchEvents.push({
      step: ++eventStep,
      type: 'DISCOVER',
      coord: { ...start },
      gScore: 0,
      fScore: startH,
      hScore: startH,
    });
  }

  let expandedCount = 0;
  let generatedCount = 0;
  let goalReached = false;

  while (!pq.isEmpty()) {
    const current = pq.pop()!;
    const currKey = coordKey(current.coord);

    if (closedSet.has(currKey)) continue;
    closedSet.add(currKey);
    expandedCount++;

    if (recordEvents) {
      searchEvents.push({
        step: ++eventStep,
        type: 'EXPAND',
        coord: { ...current.coord },
        gScore: 0,
        fScore: current.h,
        hScore: current.h,
      });
    }

    if (currKey === goalKey) {
      goalReached = true;
      if (recordEvents) {
        searchEvents.push({
          step: ++eventStep,
          type: 'GOAL_REACHED',
          coord: { ...goal },
          gScore: 0,
          fScore: 0,
          hScore: 0,
        });
      }
      break;
    }

    const neighbors = grid.getNeighbors(current.coord.r, current.coord.c);
    for (const neighbor of neighbors) {
      generatedCount++;
      const nKey = coordKey(neighbor.coord);
      if (closedSet.has(nKey) || discoveredSet.has(nKey)) continue;

      discoveredSet.add(nKey);
      parentMap.set(nKey, current.coord);

      const h = manhattanDistance(neighbor.coord, goal);
      pq.push({ coord: neighbor.coord, h }, h);

      if (recordEvents) {
        searchEvents.push({
          step: ++eventStep,
          type: 'DISCOVER',
          coord: { ...neighbor.coord },
          gScore: 0,
          fScore: h,
          hScore: h,
        });
      }
    }
  }

  const elapsed = performance.now() - startTime;

  if (!goalReached) {
    return {
      algorithm: 'greedy',
      found: false,
      path: [],
      expandedNodes: expandedCount,
      discoveredNodes: discoveredSet.size,
      generatedNodes: generatedCount,
      totalCost: 0,
      pathLength: 0,
      executionTimeMs: Number(elapsed.toFixed(3)),
      searchEvents,
      errorMessage: 'No path exists to the goal.',
      exploredCoords: Array.from(closedSet).map(parseCoordKey),
      frontierCoords: Array.from(discoveredSet)
        .filter((k) => !closedSet.has(k))
        .map(parseCoordKey),
    };
  }

  const path = reconstructPath(parentMap, start, goal);
  const totalCost = computePathCost(grid, path);

  return {
    algorithm: 'greedy',
    found: true,
    path,
    expandedNodes: expandedCount,
    discoveredNodes: discoveredSet.size,
    generatedNodes: generatedCount,
    totalCost,
    pathLength: path.length > 0 ? path.length - 1 : 0,
    executionTimeMs: Number(elapsed.toFixed(3)),
    searchEvents,
    exploredCoords: Array.from(closedSet).map(parseCoordKey),
    frontierCoords: Array.from(discoveredSet)
      .filter((k) => !closedSet.has(k))
      .map(parseCoordKey),
  };
}

/**
 * Universal dispatcher for any of the 3 algorithms.
 */
export function runAlgorithm(
  algorithm: AlgorithmType,
  grid: WarehouseGrid,
  recordEvents: boolean = true
): AlgorithmResult {
  switch (algorithm) {
    case 'astar':
      return runAStar(grid, recordEvents);
    case 'dijkstra':
      return runDijkstra(grid, recordEvents);
    case 'greedy':
      return runGreedyBestFirst(grid, recordEvents);
  }
}
