"""
Input validation and bounds checking utilities.
"""
from typing import Tuple
from src.environment.cell import Coordinate


def validate_grid_dimensions(rows: int, cols: int) -> Tuple[bool, str]:
    if not isinstance(rows, int) or not isinstance(cols, int):
        return False, "Rows and columns must be integers."
    if rows < 5 or rows > 50:
        return False, f"Rows must be between 5 and 50 (got {rows})."
    if cols < 5 or cols > 60:
        return False, f"Columns must be between 5 and 60 (got {cols})."
    return True, "Dimensions are valid."


def validate_start_goal(
    start: Coordinate, goal: Coordinate, rows: int, cols: int
) -> Tuple[bool, str]:
    if start[0] < 0 or start[0] >= rows or start[1] < 0 or start[1] >= cols:
        return False, f"Start coordinate {start} is out of bounds."
    if goal[0] < 0 or goal[0] >= rows or goal[1] < 0 or goal[1] >= cols:
        return False, f"Goal coordinate {goal} is out of bounds."
    if start == goal:
        return False, "Start and goal coordinates cannot be identical."
    return True, "Start and goal are valid."
