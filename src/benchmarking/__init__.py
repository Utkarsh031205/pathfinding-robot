"""
Benchmarking package initialization.
"""
from src.benchmarking.metrics import BenchmarkMetric
from src.benchmarking.runner import run_controlled_comparison, run_multi_scenario_experiment

__all__ = ["BenchmarkMetric", "run_controlled_comparison", "run_multi_scenario_experiment"]
