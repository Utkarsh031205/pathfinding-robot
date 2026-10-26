"""
Grid visualization rendering using Plotly and Matplotlib.
"""
from typing import List, Optional, Set
import numpy as np
import plotly.graph_objects as go
from src.environment.cell import CellType, Coordinate
from src.environment.grid import WarehouseGrid
from src.simulation.robot import Robot


# Numerical codes for visual heatmap layers
CELL_CODES = {
    CellType.EMPTY: 0,
    CellType.TERRAIN_LOW: 1,
    CellType.TERRAIN_STANDARD: 2,
    CellType.TERRAIN_HIGH: 3,
    CellType.TERRAIN_VERY_HIGH: 4,
    CellType.RESTRICTED: 5,
    CellType.WALL: 6,
    CellType.DYNAMIC_OBSTACLE: 7,
}

COLOR_SCALE = [
    [0.00, "#f8fafc"],  # 0: Empty (light clean slate)
    [0.14, "#dbeafe"],  # 1: Low-cost express (soft blue)
    [0.28, "#e2e8f0"],  # 2: Standard (light gray)
    [0.42, "#fed7aa"],  # 3: High-cost rough (amber/orange)
    [0.57, "#fca5a5"],  # 4: Very high-cost spill (deep red-orange)
    [0.71, "#cbd5e1"],  # 5: Restricted zone (cross-hatched / slate)
    [0.85, "#1e293b"],  # 6: Wall (dark steel rack)
    [1.00, "#dc2626"],  # 7: Dynamic obstacle (bright warning red)
]


def render_grid_plotly(
    grid: WarehouseGrid,
    path: Optional[List[Coordinate]] = None,
    visited: Optional[List[Coordinate]] = None,
    frontier: Optional[List[Coordinate]] = None,
    robot: Optional[Robot] = None,
    traversed_path: Optional[List[Coordinate]] = None,
    title: str = "Warehouse Navigation Grid",
) -> go.Figure:
    """
    Renders high-resolution interactive warehouse grid in Plotly.
    """
    z = np.zeros((grid.rows, grid.cols), dtype=float)
    hover_text = []

    for r in range(grid.rows):
        row_text = []
        for c in range(grid.cols):
            cell = grid.get_cell(r, c)
            code = CELL_CODES.get(cell.cell_type, 0)
            z[r, c] = code
            cost_str = "∞" if cell.cost == float("inf") else f"{cell.cost:.1f}"
            row_text.append(f"Cell ({r}, {c})<br>Type: {cell.cell_type.value}<br>Cost: {cost_str}")
        hover_text.append(row_text)

    fig = go.Figure()

    # Base grid heatmap
    fig.add_trace(
        go.Heatmap(
            z=z,
            text=hover_text,
            hoverinfo="text",
            colorscale=COLOR_SCALE,
            zmin=0,
            zmax=7,
            showscale=False,
            xgap=1,
            ygap=1,
        )
    )

    # Explored / Visited nodes overlay (subtle emerald glow)
    if visited:
        vr = [pt[0] for pt in visited if pt != grid.start and pt != grid.goal]
        vc = [pt[1] for pt in visited if pt != grid.start and pt != grid.goal]
        if vr:
            fig.add_trace(
                go.Scatter(
                    x=vc,
                    y=vr,
                    mode="markers",
                    marker=dict(symbol="square", size=14, color="rgba(16, 185, 129, 0.35)", line=dict(width=0)),
                    name="Explored Nodes",
                    hoverinfo="none",
                )
            )

    # Frontier nodes overlay (cyan dots)
    if frontier:
        fr = [pt[0] for pt in frontier if pt != grid.start and pt != grid.goal]
        fc = [pt[1] for pt in frontier if pt != grid.start and pt != grid.goal]
        if fr:
            fig.add_trace(
                go.Scatter(
                    x=fc,
                    y=fr,
                    mode="markers",
                    marker=dict(symbol="diamond", size=12, color="rgba(6, 182, 212, 0.75)", line=dict(width=1, color="#0891b2")),
                    name="Frontier",
                    hoverinfo="none",
                )
            )

    # Traversed path trail by robot (solid purple line)
    if traversed_path and len(traversed_path) > 1:
        tr_r = [pt[0] for pt in traversed_path]
        tr_c = [pt[1] for pt in traversed_path]
        fig.add_trace(
            go.Scatter(
                x=tr_c,
                y=tr_r,
                mode="lines",
                line=dict(color="#8b5cf6", width=4, dash="solid"),
                name="Robot Traversed Trail",
            )
        )

    # Planned path (goldenrod highlighted path line)
    if path and len(path) > 1:
        pr = [pt[0] for pt in path]
        pc = [pt[1] for pt in path]
        fig.add_trace(
            go.Scatter(
                x=pc,
                y=pr,
                mode="lines+markers",
                line=dict(color="#f59e0b", width=3, dash="dot"),
                marker=dict(size=6, color="#d97706"),
                name="Planned Path",
            )
        )

    # Start marker (Green circle)
    fig.add_trace(
        go.Scatter(
            x=[grid.start[1]],
            y=[grid.start[0]],
            mode="markers+text",
            marker=dict(size=18, color="#22c55e", line=dict(width=2, color="#15803d")),
            text=["START"],
            textposition="top center",
            name="Start",
        )
    )

    # Goal marker (Red flag/diamond)
    fig.add_trace(
        go.Scatter(
            x=[grid.goal[1]],
            y=[grid.goal[0]],
            mode="markers+text",
            marker=dict(size=18, color="#ef4444", symbol="diamond", line=dict(width=2, color="#b91c1c")),
            text=["GOAL"],
            textposition="top center",
            name="Goal",
        )
    )

    # Robot current position (Blue robot avatar/circle)
    if robot:
        fig.add_trace(
            go.Scatter(
                x=[robot.position[1]],
                y=[robot.position[0]],
                mode="markers+text",
                marker=dict(size=22, color="#3b82f6", symbol="circle", line=dict(width=3, color="#1d4ed8")),
                text=[f"🤖 {robot.direction.value}"],
                textposition="bottom center",
                name="Robot",
            )
        )

    fig.update_layout(
        title=dict(text=title, x=0.5, font=dict(size=16, family="sans-serif")),
        yaxis=dict(autorange="reversed", showgrid=False, zeroline=False, title="Row"),
        xaxis=dict(showgrid=False, zeroline=False, title="Column"),
        margin=dict(l=40, r=40, t=50, b=40),
        height=540,
        legend=dict(orientation="h", yanchor="bottom", y=-0.25, xanchor="center", x=0.5),
        plot_bgcolor="#ffffff",
    )

    return fig
