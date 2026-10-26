"""
Robot state and motion tracking for warehouse simulation.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple
from src.environment.cell import Coordinate


class RobotStatus(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    PATH_FOUND = "PATH_FOUND"
    MOVING = "MOVING"
    PAUSED = "PAUSED"
    REPLANNING = "REPLANNING"
    COMPLETED = "COMPLETED"
    NO_PATH_FOUND = "NO_PATH_FOUND"


class Direction(Enum):
    UP = "UP"
    DOWN = "DOWN"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    NONE = "NONE"


@dataclass
class Robot:
    position: Coordinate
    direction: Direction = Direction.RIGHT
    step_index: int = 0
    traversed_path: List[Coordinate] = field(default_factory=list)
    status: RobotStatus = RobotStatus.IDLE
    replan_count: int = 0
    total_replan_time_s: float = 0.0

    def __post_init__(self):
        if not self.traversed_path:
            self.traversed_path = [self.position]


def compute_direction(from_coord: Coordinate, to_coord: Coordinate) -> Direction:
    dr = to_coord[0] - from_coord[0]
    dc = to_coord[1] - from_coord[1]
    if dr < 0:
        return Direction.UP
    if dr > 0:
        return Direction.DOWN
    if dc < 0:
        return Direction.LEFT
    if dc > 0:
        return Direction.RIGHT
    return Direction.NONE
