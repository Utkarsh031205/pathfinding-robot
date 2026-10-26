"""
Benchmark runner executing controlled single and multi-scenario experiments.
Guarantees identical immutable environment snapshots for all algorithms.
"""
from typing import Dict, List, Tuple
import pandas as pd
from src.environment.grid import WarehouseGrid
from src.algorithms.astar import run_astar
from src.algorithms.dijkstra import run_dijkstra
from src.algorithms.greedy_best_first import run_greedy_best_first
from src.algorithms.common import SearchResult
from src.benchmarking.metrics import BenchmarkMetric


def run_controlled_comparison(
    grid: WarehouseGrid, scenario_name: str = "Current Warehouse Map"
) -> Tuple[Dict[str, SearchResult], pd.DataFrame]:
    """
    Runs A*, Dijkstra, and Greedy Best-First Search on identical immutable snapshots.
    Returns: (results_dict, comparison_dataframe)
    """
    algorithms = [
        ("A* Search", run_astar),
        ("Dijkstra's Algorithm", run_dijkstra),
        ("Greedy Best-First Search", run_greedy_best_first),
    ]

    results: Dict[str, SearchResult] = {}
    rows = []

    for name, algo_func in algorithms:
        # Immutable snapshot clone
        grid_snapshot = grid.clone()
        res = algo_func(grid_snapshot, True)
        results[name] = res

        rows.append(
            {
                "Algorithm": name,
                "Path Found": "Yes" if res.found else "No",
                "Path Length (steps)": res.path_length,
                "Total Traversal Cost": res.total_cost,
                "Expanded Nodes": res.expanded_nodes,
                "Discovered Nodes": res.discovered_nodes,
                "Generated Nodes": res.generated_nodes,
                "Execution Time (ms)": res.execution_time_ms,
            }
        )

    df = pd.DataFrame(rows)
    return results, df


def run_multi_scenario_experiment(
    num_scenarios: int = 5,
    rows: int = 20,
    cols: int = 30,
    obstacle_density: float = 0.2,
    base_seed: int = 42,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """
    Executes repeated experiments across multiple reproducible scenarios.
    Returns: (detailed_records_df, aggregated_summary_df)
    """
    algorithms = [
        ("A* Search", run_astar),
        ("Dijkstra's Algorithm", run_dijkstra),
        ("Greedy Best-First Search", run_greedy_best_first),
    ]

    all_records = []

    for i in range(num_scenarios):
        seed = base_seed + i * 101
        test_grid = WarehouseGrid(rows=rows, cols=cols)
        test_grid.generate_random_scenario(obstacle_density=obstacle_density, seed=seed)
        scenario_label = f"Scenario #{i+1} (Seed {seed})"

        for name, algo_func in algorithms:
            snap = test_grid.clone()
            res = algo_func(snap, False)
            all_records.append(
                {
                    "Scenario": scenario_label,
                    "Algorithm": name,
                    "Found": res.found,
                    "Path Length": res.path_length if res.found else None,
                    "Total Cost": res.total_cost if res.found else None,
                    "Expanded Nodes": res.expanded_nodes,
                    "Discovered Nodes": res.discovered_nodes,
                    "Generated Nodes": res.generated_nodes,
                    "Execution Time (ms)": res.execution_time_ms,
                }
            )

    detail_df = pd.DataFrame(all_records)

    # Aggregated summary
    summary_rows = []
    for name, _ in algorithms:
        sub = detail_df[detail_df["Algorithm"] == name]
        total_runs = len(sub)
        successes = sub["Found"].sum()
        success_rate = (successes / total_runs * 100.0) if total_runs > 0 else 0.0

        succ_sub = sub[sub["Found"] == True]

        summary_rows.append(
            {
                "Algorithm": name,
                "Total Runs": total_runs,
                "Success Rate (%)": round(success_rate, 1),
                "Mean Time (ms)": round(sub["Execution Time (ms)"].mean(), 3),
                "Median Time (ms)": round(sub["Execution Time (ms)"].median(), 3),
                "Mean Expanded Nodes": round(sub["Expanded Nodes"].mean(), 1),
                "Mean Path Cost": round(succ_sub["Total Cost"].mean(), 2) if len(succ_sub) > 0 else 0.0,
                "Mean Path Length": round(succ_sub["Path Length"].mean(), 1) if len(succ_sub) > 0 else 0.0,
            }
        )

    summary_df = pd.DataFrame(summary_rows)
    return detail_df, summary_df
