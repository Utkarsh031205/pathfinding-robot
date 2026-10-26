"""
Defines cell states, types, and coordinate models for the warehouse floor.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class CellType(Enum):
    EMPTY = "EMPTY"
    START = "START"
    GOAL = "GOAL"
    WALL = "WALL"
    RESTRICTED = "RESTRICTED"
    TERRAIN_LOW = "TERRAIN_LOW"
    TERRAIN_STANDARD = "TERRAIN_STANDARD"
    TERRAIN_HIGH = "TERRAIN_HIGH"
    TERRAIN_VERY_HIGH = "TERRAIN_VERY_HIGH"
    DYNAMIC_OBSTACLE = "DYNAMIC_OBSTACLE"


Coordinate = Tuple[int, int]  # (row, col)


@dataclass
class Cell:
    row: int
    col: int
    cell_type: CellType = CellType.EMPTY
    cost: float = 1.0

    @property
    def coord(self) -> Coordinate:
        return (self.row, self.col)

    def is_passable(self, restricted_as_high_cost: bool = False) -> bool:
        if self.cell_type in (CellType.WALL, CellType.DYNAMIC_OBSTACLE):
            return False
        if self.cell_type == CellType.RESTRICTED and not restricted_as_high_cost:
            return False
        return True
