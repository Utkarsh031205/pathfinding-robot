export type CellType =
  | 'EMPTY'
  | 'WALL'
  | 'RESTRICTED'
  | 'TERRAIN_LOW'
  | 'TERRAIN_STANDARD'
  | 'TERRAIN_HIGH'
  | 'TERRAIN_VERY_HIGH'
  | 'DYNAMIC_OBSTACLE';

export interface Coordinate {
  r: number;
  c: number;
}

export type MovementDirection = 'UP' | 'DOWN' | 'LEFT' | 'RIGHT' | 'NONE';

export interface CellState {
  r: number;
  c: number;
  type: CellType;
  baseCost: number; // Traversal cost to enter this cell
  isRestrictedPassable?: boolean; // Whether restricted zone is passable with penalty
}

export interface TerrainConfig {
  lowCost: number; // default: 0.5
  standardCost: number; // default: 1.0
  highCost: number; // default: 5.0
  veryHighCost: number; // default: 10.0
  restrictedCostPenalty: number; // default: 20.0
  restrictedIsBlocked: boolean; // default: true (impassable)
}

export const DEFAULT_TERRAIN_CONFIG: TerrainConfig = {
  lowCost: 0.5,
  standardCost: 1.0,
  highCost: 5.0,
  veryHighCost: 10.0,
  restrictedCostPenalty: 25.0,
  restrictedIsBlocked: true,
};

export type AlgorithmType = 'astar' | 'dijkstra' | 'greedy';

export type SearchEventType =
  | 'DISCOVER'   // Node added to frontier
  | 'EXPAND'     // Node popped from priority queue and expanded
  | 'UPDATE_G'   // Found cheaper path to node already in frontier
  | 'GOAL_REACHED'
  | 'NO_PATH';

export interface SearchEvent {
  step: number;
  type: SearchEventType;
  coord: Coordinate;
  gScore: number;
  fScore: number;
  hScore: number;
}

export interface AlgorithmResult {
  algorithm: AlgorithmType;
  found: boolean;
  path: Coordinate[];
  expandedNodes: number;
  discoveredNodes: number;
  generatedNodes: number; // Successor evaluations
  totalCost: number;
  pathLength: number;
  executionTimeMs: number;
  searchEvents: SearchEvent[];
  errorMessage?: string;
  exploredCoords: Coordinate[];
  frontierCoords: Coordinate[];
}

export type RobotStatus =
  | 'idle'
  | 'searching'
  | 'path_found'
  | 'moving'
  | 'paused'
  | 'replanning'
  | 'completed'
  | 'no_path_found';

export interface RobotState {
  position: Coordinate;
  direction: MovementDirection;
  stepIndex: number;
  traversedPath: Coordinate[];
  status: RobotStatus;
  replanCount: number;
  totalReplanTimeMs: number;
  lastReplanStep?: number;
}

export interface BenchmarkRecord {
  id: string;
  scenarioName: string;
  algorithm: AlgorithmType;
  found: boolean;
  pathLength: number;
  totalCost: number;
  expandedNodes: number;
  discoveredNodes: number;
  generatedNodes: number;
  executionTimeMs: number;
  replanCount: number;
  gridRows: number;
  gridCols: number;
  obstacleDensity: number;
  timestamp: string;
}

export interface MultiScenarioSummary {
  algorithm: AlgorithmType;
  runs: number;
  successRate: number;
  avgTimeMs: number;
  medianTimeMs: number;
  avgCost: number;
  avgExpanded: number;
  avgPathLength: number;
}
