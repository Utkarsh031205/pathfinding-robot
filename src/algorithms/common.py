"""
Common interfaces, result models, and event tracking for search algorithms.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple
from src.environment.cell import Coordinate


class SearchEventType(Enum):
    DISCOVER = "DISCOVER"       # Node added to frontier priority queue
    EXPAND = "EXPAND"           # Node popped from priority queue and processed
    UPDATE_G = "UPDATE_G"       # Cheaper route to an existing frontier node
    GOAL_SETTLED = "GOAL_SETTLED" # Optimal goal popped from priority queue
    NO_PATH = "NO_PATH"


@dataclass
class SearchEvent:
    step: int
    event_type: SearchEventType
    coord: Coordinate
    g_score: float
    f_score: float
    h_score: float


@dataclass
class SearchResult:
    """
    Standardized result contract across all pathfinding algorithms.
    """
    algorithm_name: str
    found: bool
    path: List[Coordinate]
    expanded_nodes: int
    discovered_nodes: int
    generated_nodes: int  # Total candidate successor states evaluated
    total_cost: float
    path_length: int
    execution_time: float  # In seconds
    search_events: List[SearchEvent] = field(default_factory=list)
    error_message: Optional[str] = None
    visited_nodes: List[Coordinate] = field(default_factory=list)
    frontier_nodes: List[Coordinate] = field(default_factory=list)

    @property
    def execution_time_ms(self) -> float:
        return round(self.execution_time * 1000.0, 3)


def reconstruct_path(
    parent_map: dict[Coordinate, Coordinate],
    start: Coordinate,
    goal: Coordinate,
) -> List[Coordinate]:
    """Reconstructs ordered path from start to goal via parent pointers."""
    path = []
    curr: Optional[Coordinate] = goal
    while curr is not None:
        path.append(curr)
        if curr == start:
            break
        curr = parent_map.get(curr)
    path.reverse()
    return path
