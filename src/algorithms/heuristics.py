"""
Heuristic functions for informed graph search.
Includes standard Manhattan and scaled Manhattan to preserve admissibility across weighted terrain.
"""
import math
from src.environment.cell import Coordinate


def manhattan_distance(a: Coordinate, b: Coordinate) -> float:
    """
    Standard L1 Manhattan distance on a 4-directional grid:
    h(n) = |row(a) - row(b)| + |col(a) - col(b)|
    Admissible when movement cost is uniformly 1.0.
    """
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))


def scaled_manhattan_distance(a: Coordinate, b: Coordinate, min_cost: float) -> float:
    """
    Scaled Manhattan distance:
    h(n) = Manhattan(a, b) * min_cost
    Guarantees admissibility for nonuniform terrain:
    Because every single step on the grid costs at least min_cost,
    Manhattan(a, b) * min_cost <= true optimal remaining cost h*(n).
    """
    return manhattan_distance(a, b) * min_cost


def euclidean_distance(a: Coordinate, b: Coordinate) -> float:
    """Straight-line Euclidean distance."""
    return math.hypot(a[0] - b[0], a[1] - b[1])
