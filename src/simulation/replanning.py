"""
Dynamic obstacle detection and route replanning logic.
"""
import time
from typing import Callable, List, Optional, Tuple
from src.environment.cell import CellType, Coordinate
from src.environment.grid import WarehouseGrid
from src.simulation.robot import Robot, RobotStatus, compute_direction
from src.algorithms.common import SearchResult


def replan_route(
    grid: WarehouseGrid,
    robot: Robot,
    algorithm_fn: Callable[[WarehouseGrid, bool], SearchResult],
) -> Tuple[bool, List[Coordinate], str, float]:
    """
    Replans route from robot's current position to the destination goal.
    Preserves robot position.
    Returns: (success, new_remainder_path, message, elapsed_time_s)
    """
    replan_start = time.perf_counter()

    # Sub-grid starting at robot's current position
    sub_grid = grid.clone()
    sub_grid.start = robot.position
    sub_grid.goal = grid.goal

    search_res = algorithm_fn(sub_grid, False)
    elapsed = time.perf_counter() - replan_start

    robot.replan_count += 1
    robot.total_replan_time_s += elapsed

    if not search_res.found or len(search_res.path) == 0:
        robot.status = RobotStatus.NO_PATH_FOUND
        return False, [], search_res.error_message or "No alternative route found around blockage.", elapsed

    new_path = search_res.path
    robot.status = RobotStatus.MOVING
    if len(new_path) > 1:
        robot.direction = compute_direction(robot.position, new_path[1])

    msg = f"Replan #{robot.replan_count} succeeded in {round(elapsed*1000, 2)}ms. New route has {search_res.path_length} steps."
    return True, new_path, msg, elapsed
