"""
Unit tests for Dijkstra's shortest-path algorithm.
"""
import pytest
from src.environment.cell import CellType
from src.environment.grid import WarehouseGrid
from src.algorithms.dijkstra import run_dijkstra


def test_dijkstra_straight_line():
    grid = WarehouseGrid(rows=5, cols=8, start=(2, 1), goal=(2, 6))
    res = run_dijkstra(grid)
    assert res.found
    assert res.path_length == 5
    assert res.total_cost == 5.0
    assert res.path[0] == (2, 1)
    assert res.path[-1] == (2, 6)


def test_dijkstra_detour_around_wall():
    grid = WarehouseGrid(rows=7, cols=7, start=(3, 1), goal=(3, 5))
    # Wall dividing row 3
    for r in range(1, 6):
        grid.set_cell_type(r, 3, CellType.WALL)
    # Opening at row 0
    res = run_dijkstra(grid)
    assert res.found
    # Path must detour around column 3 through row 0 or 6
    assert all(grid.is_passable(*coord) for coord in res.path)


def test_dijkstra_weighted_terrain_optimality():
    grid = WarehouseGrid(rows=6, cols=8, start=(2, 1), goal=(2, 6))
    # Middle direct row has very high cost (10.0)
    for c in range(2, 6):
        grid.set_cell_type(2, c, CellType.TERRAIN_VERY_HIGH)
    # Row 0 has low-cost conveyor (0.5)
    for c in range(1, 7):
        grid.set_cell_type(0, c, CellType.TERRAIN_LOW)

    res = run_dijkstra(grid)
    assert res.found
    # Dijkstra must find the cheaper detour path rather than traversing the 10.0 mud
    direct_cost = 4 * 10.0
    assert res.total_cost < direct_cost


def test_dijkstra_blocked_goal():
    grid = WarehouseGrid(rows=6, cols=6, start=(1, 1), goal=(4, 4))
    grid.set_cell_type(4, 4, CellType.WALL, force=True)
    res = run_dijkstra(grid)
    assert not res.found
    assert "blocked" in (res.error_message or "").lower()


def test_dijkstra_start_equals_goal():
    grid = WarehouseGrid(rows=5, cols=5, start=(2, 2), goal=(2, 2))
    res = run_dijkstra(grid)
    assert res.found
    assert res.path_length == 0
    assert res.total_cost == 0.0
