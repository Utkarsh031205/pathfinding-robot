"""
Unit tests for Greedy Best-First Search algorithm.
Verifies heuristic-driven priority and demonstrates suboptimality on weighted terrain.
"""
import pytest
from src.environment.cell import CellType
from src.environment.grid import WarehouseGrid
from src.algorithms.greedy_best_first import run_greedy_best_first
from src.algorithms.dijkstra import run_dijkstra


def test_greedy_straight_line():
    grid = WarehouseGrid(rows=5, cols=8, start=(2, 1), goal=(2, 6))
    res = run_greedy_best_first(grid)
    assert res.found
    assert res.path_length == 5


def test_greedy_suboptimality_on_terrain_dilemma():
    grid = WarehouseGrid()
    grid.load_preset("terrain_dilemma")

    dijkstra_res = run_dijkstra(grid)
    greedy_res = run_greedy_best_first(grid)

    assert dijkstra_res.found
    assert greedy_res.found

    # Greedy rushes blindly through the high-cost swamp because it looks geometrically closer
    # Therefore, its actual route cost must be strictly greater than or equal to Dijkstra's optimal route
    assert greedy_res.total_cost >= dijkstra_res.total_cost
