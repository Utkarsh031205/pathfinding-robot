"""
Metrics definitions and calculations for search algorithm benchmarking.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class BenchmarkMetric:
    scenario_name: str
    algorithm: str
    found: bool
    path_length: int
    total_cost: float
    expanded_nodes: int
    discovered_nodes: int
    generated_nodes: int
    execution_time_ms: float
    replan_count: int = 0
    grid_size: str = ""
    obstacle_density: float = 0.0
