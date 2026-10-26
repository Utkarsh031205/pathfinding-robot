"""
Unit tests for WarehouseGrid environment, boundaries, and 4-directional movement.
"""
import pytest
from src.environment.cell import CellType
from src.environment.grid import WarehouseGrid
from src.environment.terrain import TerrainConfig


def test_grid_initialization_and_bounds():
    grid = WarehouseGrid(rows=12, cols=16)
    assert grid.rows == 12
    assert grid.cols == 16
    assert grid.is_in_bounds(0, 0)
    assert grid.is_in_bounds(11, 15)
    assert not grid.is_in_bounds(-1, 0)
    assert not grid.is_in_bounds(12, 16)


def test_four_directional_movement_only():
    grid = WarehouseGrid(rows=10, cols=10)
    # Corner cell (0, 0) should have exactly 2 orthogonal neighbors
    corner_neighbors = grid.get_neighbors(0, 0)
    assert len(corner_neighbors) == 2
    directions = sorted([n[1] for n in corner_neighbors])
    assert directions == ["DOWN", "RIGHT"]

    # Interior cell (5, 5) should have exactly 4 orthogonal neighbors (no diagonals)
    center_neighbors = grid.get_neighbors(5, 5)
    assert len(center_neighbors) == 4
    center_dirs = sorted([n[1] for n in center_neighbors])
    assert center_dirs == ["DOWN", "LEFT", "RIGHT", "UP"]


def test_walls_block_movement():
    grid = WarehouseGrid(rows=8, cols=8)
    grid.set_cell_type(4, 5, CellType.WALL)
    assert not grid.is_passable(4, 5)
    neighbors = grid.get_neighbors(4, 4)
    # Coordinate (4, 5) must not appear in neighbors
    assert all(n[0] != (4, 5) for n in neighbors)


def test_terrain_costs():
    config = TerrainConfig(low_cost=0.5, standard_cost=1.0, high_cost=4.0, very_high_cost=9.0)
    grid = WarehouseGrid(rows=10, cols=10, terrain_config=config)
    grid.set_cell_type(2, 2, CellType.TERRAIN_LOW)
    grid.set_cell_type(3, 3, CellType.TERRAIN_HIGH)
    grid.set_cell_type(4, 4, CellType.TERRAIN_VERY_HIGH)

    assert grid.get_cell_cost(2, 2) == 0.5
    assert grid.get_cell_cost(3, 3) == 4.0
    assert grid.get_cell_cost(4, 4) == 9.0


def test_start_goal_validation():
    grid = WarehouseGrid(rows=10, cols=10)
    # Cannot set start on wall
    grid.set_cell_type(3, 3, CellType.WALL)
    assert not grid.set_start(3, 3)

    # Cannot set goal on start
    assert not grid.set_goal(grid.start[0], grid.start[1])


def test_clone_isolation():
    grid = WarehouseGrid(rows=8, cols=8)
    grid.set_cell_type(3, 3, CellType.WALL)
    clone = grid.clone()

    # Mutating clone must not affect original
    clone.set_cell_type(4, 4, CellType.WALL)
    assert clone.get_cell(4, 4).cell_type == CellType.WALL
    assert grid.get_cell(4, 4).cell_type == CellType.EMPTY
