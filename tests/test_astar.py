"""
Unit tests for A* Search algorithm with admissible scaled Manhattan heuristic.
"""
import pytest
from src.environment.cell import CellType
from src.environment.grid import WarehouseGrid
from src.algorithms.astar import run_astar
from src.algorithms.dijkstra import run_dijkstra


def test_astar_straight_line():
    grid = WarehouseGrid(rows=6, cols=10, start=(2, 1), goal=(2, 8))
    res = run_astar(grid)
    assert res.found
    assert res.path_length == 7
    assert res.total_cost == 7.0


def test_astar_matches_dijkstra_optimal_cost():
    grid = WarehouseGrid(rows=8, cols=10, start=(1, 1), goal=(6, 8))
    # Wall requiring detour
    for r in range(2, 6):
        grid.set_cell_type(r, 4, CellType.WALL)

    dijkstra_res = run_dijkstra(grid)
    astar_res = run_astar(grid)

    assert dijkstra_res.found
    assert astar_res.found
    # A* and Dijkstra must match the optimal cost
    assert astar_res.total_cost == dijkstra_res.total_cost
    assert astar_res.path_length == dijkstra_res.path_length

    # A* should expand fewer or equal nodes than Dijkstra
    assert astar_res.expanded_nodes <= dijkstra_res.expanded_nodes


def test_astar_weighted_terrain_optimality():
    grid = WarehouseGrid(rows=6, cols=8, start=(2, 1), goal=(2, 6))
    for c in range(2, 6):
        grid.set_cell_type(2, c, CellType.TERRAIN_VERY_HIGH)
    for c in range(1, 7):
        grid.set_cell_type(0, c, CellType.TERRAIN_LOW)

    astar_res = run_astar(grid)
    dijkstra_res = run_dijkstra(grid)

    assert astar_res.found
    assert astar_res.total_cost == dijkstra_res.total_cost


def test_astar_start_equals_goal():
    grid = WarehouseGrid(rows=4, cols=4, start=(1, 1), goal=(1, 1))
    res = run_astar(grid)
    assert res.found
    assert res.path_length == 0
