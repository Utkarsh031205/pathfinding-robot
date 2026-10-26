"""
A* Search algorithm for 2D warehouse pathfinding with admissible Manhattan heuristic.
Guarantees optimal path when heuristic is scaled by minimum movement cost.
"""
import heapq
import time
from typing import Dict, List, Set, Tuple
from src.environment.cell import Coordinate
from src.environment.grid import WarehouseGrid
from src.algorithms.common import SearchEvent, SearchEventType, SearchResult, reconstruct_path
from src.algorithms.heuristics import scaled_manhattan_distance


def run_astar(grid: WarehouseGrid, record_events: bool = True) -> SearchResult:
    """
    Executes A* Search from grid.start to grid.goal.
    Priority is f(n) = g(n) + h(n), with h(n) tie-breaking.
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
            algorithm_name="A* Search",
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
            algorithm_name="A* Search",
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
            algorithm_name="A* Search",
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

    min_cost = grid.terrain_config.min_permitted_cost()
    start_h = scaled_manhattan_distance(start, goal, min_cost)

    # Priority queue entries: (f_score, h_score, insert_order, coord)
    counter = 0
    pq: List[Tuple[float, float, int, Coordinate]] = []
    heapq.heappush(pq, (start_h, start_h, counter, start))

    g_scores: Dict[Coordinate, float] = {start: 0.0}
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
        curr_f, curr_h, _, curr_coord = heapq.heappop(pq)

        if curr_coord in closed_set:
            continue
        curr_g = g_scores.get(curr_coord, float("inf"))
        if curr_f > curr_g + curr_h + 1e-9:
            continue

        closed_set.add(curr_coord)
        expanded_count += 1

        if record_events:
            event_step += 1
            search_events.append(SearchEvent(event_step, SearchEventType.EXPAND, curr_coord, curr_g, curr_f, curr_h))

        if curr_coord == goal:
            goal_settled = True
            if record_events:
                event_step += 1
                search_events.append(SearchEvent(event_step, SearchEventType.GOAL_SETTLED, goal, curr_g, curr_f, 0.0))
            break

        for neighbor_coord, _, step_cost in grid.get_neighbors(*curr_coord):
            generated_count += 1
            if neighbor_coord in closed_set:
                continue

            tentative_g = curr_g + step_cost
            prev_g = g_scores.get(neighbor_coord, float("inf"))

            if tentative_g < prev_g:
                g_scores[neighbor_coord] = tentative_g
                parent_map[neighbor_coord] = curr_coord
                h = scaled_manhattan_distance(neighbor_coord, goal, min_cost)
                f = tentative_g + h

                counter += 1
                heapq.heappush(pq, (f, h, counter, neighbor_coord))

                is_new = neighbor_coord not in discovered_set
                discovered_set.add(neighbor_coord)

                if record_events:
                    event_step += 1
                    evt_type = SearchEventType.DISCOVER if is_new else SearchEventType.UPDATE_G
                    search_events.append(SearchEvent(event_step, evt_type, neighbor_coord, tentative_g, f, h))

    elapsed = time.perf_counter() - start_time

    if not goal_settled:
        return SearchResult(
            algorithm_name="A* Search",
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
    total_cost = g_scores[goal]

    return SearchResult(
        algorithm_name="A* Search",
        found=True,
        path=path,
        expanded_nodes=expanded_count,
        discovered_nodes=len(discovered_set),
        generated_nodes=generated_count,
        total_cost=round(total_cost, 4),
        path_length=max(0, len(path) - 1),
        execution_time=elapsed,
        search_events=search_events,
        visited_nodes=list(closed_set),
        frontier_nodes=[c for c in discovered_set if c not in closed_set],
    )
