"""
Search animation and step-by-step playback utilities.
"""
from typing import List, Set, Tuple
from src.algorithms.common import SearchEvent, SearchEventType
from src.environment.cell import Coordinate


def get_search_state_at_step(
    events: List[SearchEvent], step_index: int
) -> Tuple[List[Coordinate], List[Coordinate]]:
    """
    Given recorded search events and a target step index,
    reconstructs the set of visited (closed) coordinates and frontier coordinates.
    """
    visited: Set[Coordinate] = set()
    frontier: Set[Coordinate] = set()

    max_step = min(step_index, len(events))
    for i in range(max_step):
        evt = events[i]
        if evt.event_type == SearchEventType.DISCOVER:
            frontier.add(evt.coord)
        elif evt.event_type == SearchEventType.EXPAND:
            visited.add(evt.coord)
            frontier.discard(evt.coord)
        elif evt.event_type == SearchEventType.GOAL_SETTLED:
            visited.add(evt.coord)
            frontier.discard(evt.coord)

    return list(visited), list(frontier)
