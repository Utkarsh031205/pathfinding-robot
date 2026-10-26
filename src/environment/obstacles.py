"""
Obstacle management and warehouse floor layouts.
"""
import random
from typing import List, Set, Tuple
from src.environment.cell import CellType, Coordinate


def generate_random_walls(
    rows: int,
    cols: int,
    density: float,
    seed: int,
    start: Coordinate,
    goal: Coordinate,
) -> Set[Coordinate]:
    """
    Deterministically generates wall coordinates based on density and seed.
    Keeps immediate Manhattan distance 1 from start and goal clear to avoid trivial traps.
    """
    rng = random.Random(seed)
    walls: Set[Coordinate] = set()

    for r in range(rows):
        for c in range(cols):
            if (r, c) == start or (r, c) == goal:
                continue
            # Keep a buffer around start and goal
            if abs(r - start[0]) + abs(c - start[1]) <= 1:
                continue
            if abs(r - goal[0]) + abs(c - goal[1]) <= 1:
                continue

            if rng.random() < density:
                walls.add((r, c))

    return walls
