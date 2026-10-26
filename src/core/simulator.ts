import {
  AlgorithmType,
  Coordinate,
  MovementDirection,
  RobotState,
  RobotStatus,
} from '../types/simulator';
import { WarehouseGrid } from './grid';
import { runAlgorithm } from './algorithms';

export function computeDirection(from: Coordinate, to: Coordinate): MovementDirection {
  if (to.r < from.r) return 'UP';
  if (to.r > from.r) return 'DOWN';
  if (to.c < from.c) return 'LEFT';
  if (to.c > from.c) return 'RIGHT';
  return 'NONE';
}

export class RobotSimulator {
  public grid: WarehouseGrid;
  public algorithm: AlgorithmType;
  public state: RobotState;
  public fullPlannedPath: Coordinate[] = [];
  public originalPlannedPath: Coordinate[] = [];

  constructor(grid: WarehouseGrid, algorithm: AlgorithmType = 'astar') {
    this.grid = grid;
    this.algorithm = algorithm;
    this.state = {
      position: { ...grid.start },
      direction: 'RIGHT',
      stepIndex: 0,
      traversedPath: [{ ...grid.start }],
      status: 'idle',
      replanCount: 0,
      totalReplanTimeMs: 0,
    };
  }

  /**
   * Initializes or plans initial mission from start to goal.
   */
  public planMission(): { success: boolean; message: string; path: Coordinate[] } {
    this.state.status = 'searching';
    const result = runAlgorithm(this.algorithm, this.grid, false);

    if (!result.found || result.path.length === 0) {
      this.state.status = 'no_path_found';
      return {
        success: false,
        message: result.errorMessage ?? 'No valid path found to the goal.',
        path: [],
      };
    }

    this.fullPlannedPath = [...result.path];
    this.originalPlannedPath = [...result.path];
    this.state.position = { ...this.grid.start };
    this.state.stepIndex = 0;
    this.state.traversedPath = [{ ...this.grid.start }];
    this.state.status = 'path_found';
    this.state.replanCount = 0;
    this.state.totalReplanTimeMs = 0;

    if (result.path.length > 1) {
      this.state.direction = computeDirection(result.path[0], result.path[1]);
    }

    return {
      success: true,
      message: `Initial path found (${result.pathLength} steps, cost: ${result.totalCost})`,
      path: this.fullPlannedPath,
    };
  }

  /**
   * Advance robot by one discrete step along remaining planned path.
   */
  public stepMovement(): {
    status: RobotStatus;
    completed: boolean;
    replanned: boolean;
    error?: string;
  } {
    if (this.state.status === 'completed' || this.state.status === 'no_path_found') {
      return { status: this.state.status, completed: this.state.status === 'completed', replanned: false };
    }

    const currentIndex = this.state.stepIndex;
    const nextIndex = currentIndex + 1;

    // Check if robot has reached end of current path
    if (nextIndex >= this.fullPlannedPath.length) {
      if (
        this.state.position.r === this.grid.goal.r &&
        this.state.position.c === this.grid.goal.c
      ) {
        this.state.status = 'completed';
        return { status: 'completed', completed: true, replanned: false };
      }
      this.state.status = 'no_path_found';
      return {
        status: 'no_path_found',
        completed: false,
        replanned: false,
        error: 'Reached end of planned path without reaching goal.',
      };
    }

    const nextCoord = this.fullPlannedPath[nextIndex];

    // Check if the next cell is blocked (dynamic obstacle or wall placed)
    if (!this.grid.isPassable(nextCoord.r, nextCoord.c)) {
      // Automatic immediate replan from current position!
      const replanResult = this.triggerReplan();
      if (!replanResult.success) {
        this.state.status = 'no_path_found';
        return {
          status: 'no_path_found',
          completed: false,
          replanned: true,
          error: `Obstacle blocked path at (${nextCoord.r}, ${nextCoord.c}). Replanning failed: ${replanResult.message}`,
        };
      }
      return { status: 'moving', completed: false, replanned: true };
    }

    // Valid step: move robot
    this.state.direction = computeDirection(this.state.position, nextCoord);
    this.state.position = { ...nextCoord };
    this.state.stepIndex = nextIndex;
    this.state.traversedPath.push({ ...nextCoord });
    this.state.status = 'moving';

    // Check if goal reached after this step
    if (
      this.state.position.r === this.grid.goal.r &&
      this.state.position.c === this.grid.goal.c
    ) {
      this.state.status = 'completed';
      return { status: 'completed', completed: true, replanned: false };
    }

    return { status: 'moving', completed: false, replanned: false };
  }

  /**
   * Introduces a dynamic obstacle at (r, c).
   * Validates:
   * - Cannot place on robot current position.
   * - If on remaining path, triggers route invalidation and replanning.
   */
  public insertDynamicObstacle(r: number, c: number): {
    success: boolean;
    affectsPlannedRoute: boolean;
    replanned: boolean;
    message: string;
  } {
    if (!this.grid.isInBounds(r, c)) {
      return {
        success: false,
        affectsPlannedRoute: false,
        replanned: false,
        message: `Coordinate (${r}, ${c}) is out of bounds.`,
      };
    }

    // Safety constraint: Never place obstacle on top of the robot
    if (this.state.position.r === r && this.state.position.c === c) {
      return {
        success: false,
        affectsPlannedRoute: false,
        replanned: false,
        message: 'Cannot place obstacle directly on the robot current location.',
      };
    }

    // Goal blocked rule
    if (this.grid.isGoal(r, c)) {
      this.grid.setCell(r, c, 'DYNAMIC_OBSTACLE');
      this.state.status = 'no_path_found';
      return {
        success: true,
        affectsPlannedRoute: true,
        replanned: false,
        message: 'Dynamic obstacle placed on goal! Destination is now unreachable.',
      };
    }

    this.grid.setCell(r, c, 'DYNAMIC_OBSTACLE');

    // Check if obstacle lies on any of the future steps
    const remainingSteps = this.fullPlannedPath.slice(this.state.stepIndex + 1);
    const affectsPlannedRoute = remainingSteps.some((pt) => pt.r === r && pt.c === c);

    if (affectsPlannedRoute) {
      const replanResult = this.triggerReplan();
      return {
        success: true,
        affectsPlannedRoute: true,
        replanned: replanResult.success,
        message: replanResult.success
          ? `Obstacle placed at (${r}, ${c}) blocked current route. Replanning succeeded!`
          : `Obstacle placed at (${r}, ${c}). Replanning failed: ${replanResult.message}`,
      };
    }

    return {
      success: true,
      affectsPlannedRoute: false,
      replanned: false,
      message: `Dynamic obstacle placed at (${r}, ${c}). Route unaffected.`,
    };
  }

  /**
   * Triggers replanning from the robot's current position to the destination.
   * Preserves current robot position, measures replan latency, updates path.
   */
  public triggerReplan(): { success: boolean; message: string } {
    this.state.status = 'replanning';
    const replanStart = performance.now();

    // Create a temporary clone grid with start set to the robot's current position
    const subGrid = this.grid.clone();
    subGrid.start = { ...this.state.position };
    subGrid.goal = { ...this.grid.goal };

    // Search from robot position to goal
    const result = runAlgorithm(this.algorithm, subGrid, false);
    const replanElapsed = performance.now() - replanStart;

    this.state.replanCount++;
    this.state.totalReplanTimeMs += replanElapsed;
    this.state.lastReplanStep = this.state.stepIndex;

    if (!result.found || result.path.length === 0) {
      this.state.status = 'no_path_found';
      return {
        success: false,
        message: result.errorMessage ?? 'No alternate route exists around obstacle.',
      };
    }

    // Merge traversed history up to current step with the new remainder path
    const traversedSlice = this.fullPlannedPath.slice(0, this.state.stepIndex + 1);
    const newRemainder = result.path.slice(1); // skip duplicate start cell

    this.fullPlannedPath = [...traversedSlice, ...newRemainder];
    this.state.status = 'moving';

    if (newRemainder.length > 0) {
      this.state.direction = computeDirection(this.state.position, newRemainder[0]);
    }

    return {
      success: true,
      message: `Replan #${this.state.replanCount} successful in ${replanElapsed.toFixed(2)}ms. New path length: ${this.fullPlannedPath.length - 1} steps.`,
    };
  }

  public resetSimulation(): void {
    this.state = {
      position: { ...this.grid.start },
      direction: 'RIGHT',
      stepIndex: 0,
      traversedPath: [{ ...this.grid.start }],
      status: 'idle',
      replanCount: 0,
      totalReplanTimeMs: 0,
    };
    this.fullPlannedPath = [];
    this.originalPlannedPath = [];
  }
}
