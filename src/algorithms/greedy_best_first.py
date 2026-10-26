"""
Greedy Best-First Search algorithm for 2D warehouse grid.
Prioritizes nodes purely based on heuristic distance to goal: priority(n) = h(n).
Does not consider accumulated path cost g(n). Often fast but not optimal.
"""
import heapq
import time
from typing import Dict, List, Set, Tuple
from src.environment.cell import Coordinate
from src.environment.grid import WarehouseGrid
from src.algorithms.common import SearchEvent, SearchEventType, SearchResult, reconstruct_path
from src.algorithms.heuristics import manhattan_distance


def run_greedy_best_first(grid: WarehouseGrid, record_events: bool = True) -> SearchResult:
    """
    Executes Greedy Best-First Search from grid.start to grid.goal.
    Priority is purely h(n).
    """
    start_time = time.perf_counter()
    start = grid.start
    goal = grid.goal

    search_events: List[SearchEvent] = []
    event_step = 0

    if start == goal:
        elapsed = time.perf_counter() - start_time
        if record_events:
            search_events.append(SearchEvent(1, SearchEventType.GOAL_SETTLED, start, 0.0, 0.0, 0.0))
        return SearchResult(
            algorithm_name="Greedy Best-First Search",
            found=True,
            path=[start],
            expanded_nodes=1,
            discovered_nodes=1,
            generated_nodes=0,
            total_cost=0.0,
            path_length=0,
            execution_time=elapsed,
            search_events=search_events,
            visited_nodes=[start],
            frontier_nodes=[],
        )

    if not grid.is_passable(*start):
        return SearchResult(
            algorithm_name="Greedy Best-First Search",
            found=False,
            path=[],
            expanded_nodes=0,
            discovered_nodes=0,
            generated_nodes=0,
            total_cost=0.0,
            path_length=0,
            execution_time=time.perf_counter() - start_time,
            error_message="Start position is blocked or impassable.",
        )

    if not grid.is_passable(*goal):
        return SearchResult(
            algorithm_name="Greedy Best-First Search",
            found=False,
            path=[],
            expanded_nodes=0,
            discovered_nodes=0,
            generated_nodes=0,
            total_cost=0.0,
            path_length=0,
            execution_time=time.perf_counter() - start_time,
            error_message="Goal position is blocked or impassable.",
        )

    start_h = manhattan_distance(start, goal)
    counter = 0
    pq: List[Tuple[float, int, Coordinate]] = []
    heapq.heappush(pq, (start_h, counter, start))

    parent_map: Dict[Coordinate, Coordinate] = {}
    closed_set: Set[Coordinate] = set()
    discovered_set: Set[Coordinate] = {start}

    if record_events:
        event_step += 1
        search_events.append(SearchEvent(event_step, SearchEventType.DISCOVER, start, 0.0, start_h, start_h))

    expanded_count = 0
    generated_count = 0
    goal_settled = False

    while pq:
        curr_h, _, curr_coord = heapq.heappop(pq)

        if curr_coord in closed_set:
            continue

        closed_set.add(curr_coord)
        expanded_count += 1

        if record_events:
            event_step += 1
            search_events.append(SearchEvent(event_step, SearchEventType.EXPAND, curr_coord, 0.0, curr_h, curr_h))

        if curr_coord == goal:
            goal_settled = True
            if record_events:
                event_step += 1
                search_events.append(SearchEvent(event_step, SearchEventType.GOAL_SETTLED, goal, 0.0, 0.0, 0.0))
            break

        for neighbor_coord, _, _ in grid.get_neighbors(*curr_coord):
            generated_count += 1
            if neighbor_coord in closed_set or neighbor_coord in discovered_set:
                continue

            discovered_set.add(neighbor_coord)
            parent_map[neighbor_coord] = curr_coord
            h = manhattan_distance(neighbor_coord, goal)

            counter += 1
            heapq.heappush(pq, (h, counter, neighbor_coord))

            if record_events:
                event_step += 1
                search_events.append(SearchEvent(event_step, SearchEventType.DISCOVER, neighbor_coord, 0.0, h, h))

    elapsed = time.perf_counter() - start_time

    if not goal_settled:
        return SearchResult(
            algorithm_name="Greedy Best-First Search",
            found=False,
            path=[],
            expanded_nodes=expanded_count,
            discovered_nodes=len(discovered_set),
            generated_nodes=generated_count,
            total_cost=0.0,
            path_length=0,
            execution_time=elapsed,
            search_events=search_events,
            error_message="No passable path exists from start to goal.",
            visited_nodes=list(closed_set),
            frontier_nodes=[c for c in discovered_set if c not in closed_set],
        )

    path = reconstruct_path(parent_map, start, goal)

    # Compute actual true traversal cost of the route
    true_cost = 0.0
    for i in range(1, len(path)):
        true_cost += grid.get_cell_cost(*path[i])

    return SearchResult(
        algorithm_name="Greedy Best-First Search",
        found=True,
        path=path,
        expanded_nodes=expanded_count,
        discovered_nodes=len(discovered_set),
        generated_nodes=generated_count,
        total_cost=round(true_cost, 4),
        path_length=max(0, len(path) - 1),
        execution_time=elapsed,
        search_events=search_events,
        visited_nodes=list(closed_set),
        frontier_nodes=[c for c in discovered_set if c not in closed_set],
    )
