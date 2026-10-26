"""
Comparative charts for algorithm benchmarking.
"""
from typing import Dict
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def create_comparison_charts(df: pd.DataFrame) -> go.Figure:
    """
    Creates a 2x2 multi-panel subplot comparing:
    1. Total Traversal Cost
    2. Nodes Expanded
    3. Path Length
    4. Execution Time (ms)
    """
    fig = make_subplots(
        rows=2,
        cols=2,
        subplot_titles=(
            "Total Traversal Cost (Lower is Better)",
            "Nodes Expanded / Search Effort",
            "Path Length in Steps",
            "Execution Time (ms)",
        ),
        vertical_spacing=0.25,
        horizontal_spacing=0.15,
    )

    colors = ["#3b82f6", "#10b981", "#f59e0b"]

    # 1. Traversal Cost
    fig.add_trace(
        go.Bar(
            x=df["Algorithm"],
            y=df["Total Traversal Cost"],
            marker_color=colors,
            name="Traversal Cost",
            text=[f"{v:.1f}" for v in df["Total Traversal Cost"]],
            textposition="auto",
        ),
        row=1,
        col=1,
    )

    # 2. Nodes Expanded
    fig.add_trace(
        go.Bar(
            x=df["Algorithm"],
            y=df["Expanded Nodes"],
            marker_color=colors,
            name="Expanded Nodes",
            text=df["Expanded Nodes"],
            textposition="auto",
        ),
        row=1,
        col=2,
    )

    # 3. Path Length
    fig.add_trace(
        go.Bar(
            x=df["Algorithm"],
            y=df["Path Length (steps)"],
            marker_color=colors,
            name="Path Length",
            text=df["Path Length (steps)"],
            textposition="auto",
        ),
        row=2,
        col=1,
    )

    # 4. Execution Time
    fig.add_trace(
        go.Bar(
            x=df["Algorithm"],
            y=df["Execution Time (ms)"],
            marker_color=colors,
            name="Execution Time (ms)",
            text=[f"{v:.2f}ms" for v in df["Execution Time (ms)"]],
            textposition="auto",
        ),
        row=2,
        col=2,
    )

    fig.update_layout(
        height=620,
        showlegend=False,
        margin=dict(l=30, r=30, t=50, b=30),
        font=dict(family="sans-serif", size=12),
        plot_bgcolor="#f8fafc",
    )

    return fig
