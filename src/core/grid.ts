import {
  CellType,
  Coordinate,
  MovementDirection,
  TerrainConfig,
  DEFAULT_TERRAIN_CONFIG,
} from '../types/simulator';

export interface GridDimensions {
  rows: number;
  cols: number;
}

export class WarehouseGrid {
  public rows: number;
  public cols: number;
  public cells: CellType[][];
  public start: Coordinate;
  public goal: Coordinate;
  public terrainConfig: TerrainConfig;

  constructor(
    rows: number = 20,
    cols: number = 30,
    terrainConfig: TerrainConfig = DEFAULT_TERRAIN_CONFIG,
    start?: Coordinate,
    goal?: Coordinate
  ) {
    this.rows = Math.max(5, Math.min(50, rows));
    this.cols = Math.max(5, Math.min(60, cols));
    this.terrainConfig = { ...terrainConfig };

    this.cells = Array.from({ length: this.rows }, () =>
      Array.from({ length: this.cols }, () => 'EMPTY' as CellType)
    );

    this.start = start && this.isInBounds(start.r, start.c)
      ? { ...start }
      : { r: 1, c: 1 };
    this.goal = goal && this.isInBounds(goal.r, goal.c)
      ? { ...goal }
      : { r: this.rows - 2, c: this.cols - 2 };
  }

  public isInBounds(r: number, c: number): boolean {
    return r >= 0 && r < this.rows && c >= 0 && c < this.cols;
  }

  public getCell(r: number, c: number): CellType {
    if (!this.isInBounds(r, c)) {
      throw new Error(`Coordinate (${r}, ${c}) is out of bounds for grid ${this.rows}x${this.cols}`);
    }
    return this.cells[r][c];
  }

  public setCell(r: number, c: number, type: CellType, force: boolean = false): boolean {
    if (!this.isInBounds(r, c)) return false;

    // Do not overwrite start or goal directly with impassable blocks without moving them unless forced
    if (!force && (this.isStart(r, c) || this.isGoal(r, c))) {
      if (type === 'WALL' || type === 'DYNAMIC_OBSTACLE') {
        return false;
      }
    }

    this.cells[r][c] = type;
    return true;
  }

  public isStart(r: number, c: number): boolean {
    return this.start.r === r && this.start.c === c;
  }

  public isGoal(r: number, c: number): boolean {
    return this.goal.r === r && this.goal.c === c;
  }

  public setStart(r: number, c: number): boolean {
    if (!this.isInBounds(r, c)) return false;
    if (this.isGoal(r, c)) return false;

    const currentType = this.getCell(r, c);
    if (currentType === 'WALL' || currentType === 'DYNAMIC_OBSTACLE') {
      return false;
    }
    if (currentType === 'RESTRICTED' && this.terrainConfig.restrictedIsBlocked) {
      return false;
    }

    this.start = { r, c };
    return true;
  }

  public setGoal(r: number, c: number): boolean {
    if (!this.isInBounds(r, c)) return false;
    if (this.isStart(r, c)) return false;

    const currentType = this.getCell(r, c);
    if (currentType === 'WALL' || currentType === 'DYNAMIC_OBSTACLE') {
      return false;
    }
    if (currentType === 'RESTRICTED' && this.terrainConfig.restrictedIsBlocked) {
      return false;
    }

    this.goal = { r, c };
    return true;
  }

  public isPassable(r: number, c: number): boolean {
    if (!this.isInBounds(r, c)) return false;
    const type = this.cells[r][c];
    if (type === 'WALL' || type === 'DYNAMIC_OBSTACLE') return false;
    if (type === 'RESTRICTED' && this.terrainConfig.restrictedIsBlocked) return false;
    return true;
  }

  /**
   * Cost convention: Destination cell traversal cost.
   * Moving from any cell to (r, c) costs getCellCost(r, c).
   */
  public getCellCost(r: number, c: number): number {
    if (!this.isInBounds(r, c)) return Infinity;
    const type = this.cells[r][c];

    switch (type) {
      case 'WALL':
      case 'DYNAMIC_OBSTACLE':
        return Infinity;
      case 'RESTRICTED':
        return this.terrainConfig.restrictedIsBlocked
          ? Infinity
          : this.terrainConfig.restrictedCostPenalty;
      case 'TERRAIN_LOW':
        return this.terrainConfig.lowCost;
      case 'TERRAIN_STANDARD':
      case 'EMPTY':
        return this.terrainConfig.standardCost;
      case 'TERRAIN_HIGH':
        return this.terrainConfig.highCost;
      case 'TERRAIN_VERY_HIGH':
        return this.terrainConfig.veryHighCost;
      default:
        return this.terrainConfig.standardCost;
    }
  }

  /**
   * Returns minimum possible traversal cost of any passable cell in the current configuration.
   * Used for A* heuristic scaling to guarantee admissibility: h(n) = Manhattan(n, goal) * minCost.
   */
  public getMinMovementCost(): number {
    return Math.min(
      this.terrainConfig.lowCost,
      this.terrainConfig.standardCost,
      this.terrainConfig.highCost,
      this.terrainConfig.veryHighCost
    );
  }

  /**
   * 4-directional orthogonal neighbors (UP, DOWN, LEFT, RIGHT).
   * Strict adherence: No diagonals, boundary constrained.
   */
  public getNeighbors(r: number, c: number): { coord: Coordinate; direction: MovementDirection; cost: number }[] {
    const deltas: [number, number, MovementDirection][] = [
      [-1, 0, 'UP'],
      [1, 0, 'DOWN'],
      [0, -1, 'LEFT'],
      [0, 1, 'RIGHT'],
    ];

    const neighbors: { coord: Coordinate; direction: MovementDirection; cost: number }[] = [];

    for (const [dr, dc, dir] of deltas) {
      const nr = r + dr;
      const nc = c + dc;
      if (this.isInBounds(nr, nc) && this.isPassable(nr, nc)) {
        neighbors.push({
          coord: { r: nr, c: nc },
          direction: dir,
          cost: this.getCellCost(nr, nc),
        });
      }
    }

    return neighbors;
  }

  public clone(): WarehouseGrid {
    const newGrid = new WarehouseGrid(
      this.rows,
      this.cols,
      this.terrainConfig,
      this.start,
      this.goal
    );
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        newGrid.cells[r][c] = this.cells[r][c];
      }
    }
    return newGrid;
  }

  public clearObstaclesAndTerrain(): void {
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        this.cells[r][c] = 'EMPTY';
      }
    }
  }

  public clearDynamicObstacles(): void {
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (this.cells[r][c] === 'DYNAMIC_OBSTACLE') {
          this.cells[r][c] = 'EMPTY';
        }
      }
    }
  }

  /**
   * Generates random obstacles with a deterministic seed support.
   */
  public generateRandomObstacles(density: number = 0.2, seed: number = 42): void {
    const prng = createPRNG(seed);
    this.clearObstaclesAndTerrain();

    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (this.isStart(r, c) || this.isGoal(r, c)) continue;
        // Keep a clear 1-cell radius around start and goal to prevent trivial entrapment
        if (Math.abs(r - this.start.r) + Math.abs(c - this.start.c) <= 1) continue;
        if (Math.abs(r - this.goal.r) + Math.abs(c - this.goal.c) <= 1) continue;

        const val = prng();
        if (val < density) {
          this.cells[r][c] = 'WALL';
        }
      }
    }
  }

  /**
   * Generates a randomized terrain layout with express lanes, high traffic, and restricted zones.
   */
  public generateRandomTerrain(seed: number = 101): void {
    const prng = createPRNG(seed);
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (this.isStart(r, c) || this.isGoal(r, c)) continue;
        if (this.cells[r][c] === 'WALL') continue;

        const val = prng();
        if (val < 0.15) {
          this.cells[r][c] = 'TERRAIN_LOW'; // express lane
        } else if (val < 0.3) {
          this.cells[r][c] = 'TERRAIN_HIGH'; // rough floor
        } else if (val < 0.38) {
          this.cells[r][c] = 'TERRAIN_VERY_HIGH'; // spill zone
        } else if (val < 0.45) {
          this.cells[r][c] = 'RESTRICTED';
        } else {
          this.cells[r][c] = 'EMPTY';
        }
      }
    }
  }

  /**
   * Loads realistic industrial warehouse presets.
   */
  public loadPreset(presetName: WarehousePresetName): void {
    this.clearObstaclesAndTerrain();

    switch (presetName) {
      case 'amazon_sortation':
        // Modern logistics sortation center with long parallel shelf racks, cross-aisles, express center conveyor
        this.rows = 20;
        this.cols = 30;
        this.cells = Array.from({ length: 20 }, () => Array(30).fill('EMPTY'));
        this.start = { r: 1, c: 1 };
        this.goal = { r: 18, c: 28 };

        // Central express conveyor lane (low cost)
        for (let c = 2; c < 28; c++) {
          this.cells[10][c] = 'TERRAIN_LOW';
        }

        // Parallel storage racks (walls) with picking gaps
        for (let c = 4; c < 26; c += 4) {
          for (let r = 2; r < 9; r++) {
            if (r !== 5) this.cells[r][c] = 'WALL';
          }
          for (let r = 12; r < 19; r++) {
            if (r !== 15) this.cells[r][c] = 'WALL';
          }
        }

        // Heavy pallet staging zones (high cost)
        for (let r = 3; r <= 7; r++) {
          this.cells[r][2] = 'TERRAIN_HIGH';
          this.cells[r][27] = 'TERRAIN_HIGH';
        }

        // Hazardous chemical restricted zone
        this.cells[6][14] = 'RESTRICTED';
        this.cells[6][15] = 'RESTRICTED';
        this.cells[7][14] = 'RESTRICTED';
        this.cells[7][15] = 'RESTRICTED';
        break;

      case 'narrow_aisle':
        // High density narrow aisle warehouse
        this.rows = 20;
        this.cols = 25;
        this.cells = Array.from({ length: 20 }, () => Array(25).fill('EMPTY'));
        this.start = { r: 2, c: 1 };
        this.goal = { r: 17, c: 23 };

        for (let r = 2; r < 18; r += 3) {
          for (let c = 3; c < 22; c++) {
            if (c % 7 !== 0) {
              this.cells[r][c] = 'WALL';
            }
          }
        }
        // Spill area
        this.cells[10][10] = 'TERRAIN_VERY_HIGH';
        this.cells[10][11] = 'TERRAIN_VERY_HIGH';
        this.cells[11][10] = 'TERRAIN_VERY_HIGH';
        break;

      case 'chokepoint':
        // Warehouse with dividing wall and single bottleneck door
        this.rows = 18;
        this.cols = 24;
        this.cells = Array.from({ length: 18 }, () => Array(24).fill('EMPTY'));
        this.start = { r: 9, c: 2 };
        this.goal = { r: 9, c: 21 };

        // Dividing wall down column 12 with narrow 2-cell opening
        for (let r = 0; r < 18; r++) {
          if (r !== 8 && r !== 9) {
            this.cells[r][12] = 'WALL';
          }
        }
        // Expensive detour vs direct chokepoint
        for (let r = 6; r <= 11; r++) {
          this.cells[r][11] = 'TERRAIN_HIGH';
        }
        break;

      case 'terrain_dilemma':
        // Short high-cost direct path vs longer low-cost bypass path
        // Perfect to demonstrate Dijkstra/A* choosing optimal path vs Greedy taking expensive straight line!
        this.rows = 15;
        this.cols = 25;
        this.cells = Array.from({ length: 15 }, () => Array(25).fill('EMPTY'));
        this.start = { r: 7, c: 2 };
        this.goal = { r: 7, c: 22 };

        // Massive very high cost swamp / spill in the center direct line
        for (let r = 5; r <= 9; r++) {
          for (let c = 7; c <= 17; c++) {
            this.cells[r][c] = 'TERRAIN_VERY_HIGH'; // cost 10
          }
        }
        // Smooth express bypass at row 2 and row 12 (cost 0.5)
        for (let c = 3; c <= 21; c++) {
          this.cells[2][c] = 'TERRAIN_LOW';
        }
        break;

      case 'default':
      default:
        this.rows = 20;
        this.cols = 30;
        this.cells = Array.from({ length: 20 }, () => Array(30).fill('EMPTY'));
        this.start = { r: 2, c: 2 };
        this.goal = { r: 17, c: 27 };

        // Standard warehouse blocks
        for (let r = 4; r <= 8; r++) {
          this.cells[r][8] = 'WALL';
          this.cells[r][16] = 'WALL';
        }
        for (let r = 11; r <= 15; r++) {
          this.cells[r][8] = 'WALL';
          this.cells[r][16] = 'WALL';
          this.cells[r][22] = 'WALL';
        }
        // Express lane
        for (let c = 4; c < 26; c++) {
          this.cells[10][c] = 'TERRAIN_LOW';
        }
        // Restricted zone
        this.cells[5][12] = 'RESTRICTED';
        this.cells[5][13] = 'RESTRICTED';
        this.cells[6][12] = 'RESTRICTED';
        this.cells[6][13] = 'RESTRICTED';
        break;
    }
  }
}

export type WarehousePresetName =
  | 'default'
  | 'amazon_sortation'
  | 'narrow_aisle'
  | 'chokepoint'
  | 'terrain_dilemma';

/**
 * Deterministic pseudo-random number generator (Mulberry32)
 */
export function createPRNG(seed: number): () => number {
  let s = Math.floor(seed) >>> 0;
  return function () {
    s |= 0;
    s = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
