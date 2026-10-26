"""
Integrated simulation engine coordinating grid environment, search algorithms, and robot motion.
"""
from typing import Callable, List, Optional, Tuple
from src.environment.cell import CellType, Coordinate
from src.environment.grid import WarehouseGrid
from src.simulation.robot import Robot, RobotStatus, Direction, compute_direction
from src.simulation.replanning import replan_route
from src.algorithms.common import SearchResult
from src.algorithms.astar import run_astar
from src.algorithms.dijkstra import run_dijkstra
from src.algorithms.greedy_best_first import run_greedy_best_first


class SimulationEngine:
    """
    Coordinates path planning, step-by-step robot movement, dynamic obstacle insertion, and replanning.
    """

    def __init__(self, grid: WarehouseGrid, algorithm_name: str = "astar"):
        self.grid = grid
        self.algorithm_name = algorithm_name
        self.robot = Robot(position=grid.start)
        self.planned_path: List[Coordinate] = []
        self.original_path: List[Coordinate] = []
        self.last_search_result: Optional[SearchResult] = None

    def _get_algo_func(self) -> Callable[[WarehouseGrid, bool], SearchResult]:
        if self.algorithm_name.lower() in ("astar", "a*", "a_star"):
            return run_astar
        if self.algorithm_name.lower() in ("dijkstra", "dijkstra's"):
            return run_dijkstra
        return run_greedy_best_first

    def plan_initial_route(self) -> Tuple[bool, str]:
        """Runs the selected algorithm from start to goal."""
        self.robot.status = RobotStatus.SEARCHING
        algo_fn = self._get_algo_func()
        res = algo_fn(self.grid, True)
        self.last_search_result = res

        if not res.found or len(res.path) == 0:
            self.robot.status = RobotStatus.NO_PATH_FOUND
            return False, res.error_message or "Path planning failed."

        self.planned_path = list(res.path)
        self.original_path = list(res.path)
        self.robot.position = self.grid.start
        self.robot.step_index = 0
        self.robot.traversed_path = [self.grid.start]
        self.robot.status = RobotStatus.PATH_FOUND
        self.robot.replan_count = 0
        self.robot.total_replan_time_s = 0.0

        if len(self.planned_path) > 1:
            self.robot.direction = compute_direction(self.planned_path[0], self.planned_path[1])

        return True, f"Planned route with {res.path_length} steps (cost: {res.total_cost:.2f})"

    def step(self) -> Tuple[RobotStatus, bool, Optional[str]]:
        """
        Advances the robot by one step along the route.
        Returns: (status, reached_goal, error_message)
        """
        if self.robot.status in (RobotStatus.COMPLETED, RobotStatus.NO_PATH_FOUND):
            return self.robot.status, self.robot.status == RobotStatus.COMPLETED, None

        curr_idx = self.robot.step_index
        next_idx = curr_idx + 1

        if next_idx >= len(self.planned_path):
            if self.robot.position == self.grid.goal:
                self.robot.status = RobotStatus.COMPLETED
                return RobotStatus.COMPLETED, True, None
            self.robot.status = RobotStatus.NO_PATH_FOUND
            return RobotStatus.NO_PATH_FOUND, False, "Reached end of path without arriving at goal."

        next_coord = self.planned_path[next_idx]

        # Check for blockage in the immediate next cell
        if not self.grid.is_passable(*next_coord):
            # Immediate replan from current location
            self.robot.status = RobotStatus.REPLANNING
            success, new_remainder, msg, _ = replan_route(self.grid, self.robot, self._get_algo_func())
            if not success:
                self.robot.status = RobotStatus.NO_PATH_FOUND
                return RobotStatus.NO_PATH_FOUND, False, f"Obstacle at {next_coord} blocked path. Replanning failed."

            # Splice path
            traversed = self.planned_path[: self.robot.step_index + 1]
            self.planned_path = traversed + new_remainder[1:]
            return RobotStatus.MOVING, False, "Replanned successfully after detecting blockage."

        # Execute movement
        self.robot.direction = compute_direction(self.robot.position, next_coord)
        self.robot.position = next_coord
        self.robot.step_index = next_idx
        self.robot.traversed_path.append(next_coord)
        self.robot.status = RobotStatus.MOVING

        if self.robot.position == self.grid.goal:
            self.robot.status = RobotStatus.COMPLETED
            return RobotStatus.COMPLETED, True, None

        return RobotStatus.MOVING, False, None

    def insert_dynamic_obstacle(self, r: int, c: int) -> Tuple[bool, bool, str]:
        """
        Inserts dynamic obstacle at (r, c).
        Returns: (inserted, triggered_replan, message)
        """
        if not self.grid.is_in_bounds(r, c):
            return False, False, f"Coordinate ({r}, {c}) is outside bounds."

        if (r, c) == self.robot.position:
            return False, False, "Safety restriction: Cannot drop obstacle on robot's current position."

        if (r, c) == self.grid.goal:
            self.grid.set_cell_type(r, c, CellType.DYNAMIC_OBSTACLE, force=True)
            self.robot.status = RobotStatus.NO_PATH_FOUND
            return True, True, "Dynamic obstacle placed on goal. Goal is now unreachable!"

        self.grid.set_cell_type(r, c, CellType.DYNAMIC_OBSTACLE, force=True)

        # Check if obstacle lies on remaining route
        remaining = self.planned_path[self.robot.step_index + 1 :]
        if (r, c) in remaining:
            self.robot.status = RobotStatus.REPLANNING
            success, new_remainder, msg, _ = replan_route(self.grid, self.robot, self._get_algo_func())
            if success:
                traversed = self.planned_path[: self.robot.step_index + 1]
                self.planned_path = traversed + new_remainder[1:]
                return True, True, f"Obstacle placed at ({r}, {c}) blocked route. Replanning succeeded!"
            else:
                self.robot.status = RobotStatus.NO_PATH_FOUND
                return True, True, f"Obstacle placed at ({r}, {c}) trapped robot! No alternate route."

        return True, False, f"Dynamic obstacle placed at ({r}, {c}). Current route remains open."

    def reset(self) -> None:
        self.robot = Robot(position=self.grid.start)
        self.planned_path = []
        self.original_path = []
        self.last_search_result = None
