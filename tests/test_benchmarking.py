"""
Unit tests for comparative benchmarking and batch experiments.
"""
import pytest
from src.environment.grid import WarehouseGrid
from src.benchmarking.runner import run_controlled_comparison, run_multi_scenario_experiment


def test_controlled_comparison_snapshot_isolation():
    grid = WarehouseGrid(rows=10, cols=10)
    results, df = run_controlled_comparison(grid, "Test Map")

    assert "A* Search" in results
    assert "Dijkstra's Algorithm" in results
    assert "Greedy Best-First Search" in results

    assert len(df) == 3
    assert set(df["Algorithm"]) == {
        "A* Search",
        "Dijkstra's Algorithm",
        "Greedy Best-First Search",
    }


def test_multi_scenario_reproducibility():
    detail_df_1, summary_1 = run_multi_scenario_experiment(
        num_scenarios=2, rows=8, cols=8, obstacle_density=0.15, base_seed=99
    )
    detail_df_2, summary_2 = run_multi_scenario_experiment(
        num_scenarios=2, rows=8, cols=8, obstacle_density=0.15, base_seed=99
    )

    # Identical seed should produce identical expanded node counts and path lengths
    assert list(detail_df_1["Expanded Nodes"]) == list(detail_df_2["Expanded Nodes"])
    assert list(detail_df_1["Path Length"]) == list(detail_df_2["Path Length"])
