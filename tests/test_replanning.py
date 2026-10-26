"""
Unit tests for dynamic obstacle insertion and route replanning.
"""
import pytest
from src.environment.cell import CellType
from src.environment.grid import WarehouseGrid
from src.simulation.simulator import SimulationEngine
from src.simulation.robot import RobotStatus


def test_dynamic_obstacle_triggers_replan():
    grid = WarehouseGrid(rows=6, cols=10, start=(2, 1), goal=(2, 8))
    engine = SimulationEngine(grid, algorithm_name="astar")
    success, _ = engine.plan_initial_route()
    assert success
    assert engine.robot.status == RobotStatus.PATH_FOUND

    # Step robot 2 times
    engine.step()
    engine.step()
    assert engine.robot.position == (2, 3)

    # Insert dynamic obstacle at (2, 5) which is directly ahead on path
    inserted, replanned, msg = engine.insert_dynamic_obstacle(2, 5)
    assert inserted
    assert replanned
    assert engine.robot.replan_count == 1
    # Robot position must be preserved at (2, 3)
    assert engine.robot.position == (2, 3)

    # Continue running simulation until completion
    while engine.robot.status == RobotStatus.MOVING:
        engine.step()
        # Robot must NEVER step onto the dynamic obstacle (2, 5)
        assert engine.robot.position != (2, 5)

    assert engine.robot.status == RobotStatus.COMPLETED
    assert engine.robot.position == (2, 8)


def test_cannot_drop_obstacle_on_robot():
    grid = WarehouseGrid(rows=5, cols=5, start=(1, 1), goal=(3, 3))
    engine = SimulationEngine(grid)
    engine.plan_initial_route()

    inserted, _, _ = engine.insert_dynamic_obstacle(1, 1)
    assert not inserted


def test_dynamic_obstacle_on_goal():
    grid = WarehouseGrid(rows=5, cols=5, start=(1, 1), goal=(3, 3))
    engine = SimulationEngine(grid)
    engine.plan_initial_route()

    inserted, replanned, _ = engine.insert_dynamic_obstacle(3, 3)
    assert inserted
    assert engine.robot.status == RobotStatus.NO_PATH_FOUND
