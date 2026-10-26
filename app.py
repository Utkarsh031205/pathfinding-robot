"""
Autonomous Robot Navigation Simulator - Streamlit Application
Interactive educational dashboard for A*, Dijkstra, and Greedy Best-First Search.
"""
import time
import pandas as pd
import streamlit as st

from src.environment.cell import CellType, Coordinate
from src.environment.grid import WarehouseGrid
from src.environment.terrain import TerrainConfig
from src.simulation.simulator import SimulationEngine
from src.simulation.robot import RobotStatus
from src.algorithms.astar import run_astar
from src.algorithms.dijkstra import run_dijkstra
from src.algorithms.greedy_best_first import run_greedy_best_first
from src.benchmarking.runner import run_controlled_comparison, run_multi_scenario_experiment
from src.visualization.grid_renderer import render_grid_plotly
from src.visualization.charts import create_comparison_charts
from src.visualization.animation import get_search_state_at_step
from src.utils.validation import validate_grid_dimensions, validate_start_goal


# Streamlit Page Config
st.set_page_config(
    page_title="Autonomous Robot Navigation Simulator",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)


# Initialize Session State
def init_session_state():
    if "grid" not in st.session_state:
        grid = WarehouseGrid(rows=20, cols=30)
        grid.load_preset("amazon_sortation")
        st.session_state.grid = grid

    if "sim_engine" not in st.session_state:
        st.session_state.sim_engine = SimulationEngine(
            st.session_state.grid, algorithm_name="astar"
        )

    if "search_result" not in st.session_state:
        st.session_state.search_result = None

    if "animation_step" not in st.session_state:
        st.session_state.animation_step = 0

    if "benchmark_df" not in st.session_state:
        st.session_state.benchmark_df = None

    if "batch_detail_df" not in st.session_state:
        st.session_state.batch_detail_df = None

    if "batch_summary_df" not in st.session_state:
        st.session_state.batch_summary_df = None


init_session_state()
grid: WarehouseGrid = st.session_state.grid
sim: SimulationEngine = st.session_state.sim_engine


# --- Header ---
st.markdown(
    """
    <div style="display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid #e2e8f0; padding-bottom: 12px; margin-bottom: 18px;">
        <div>
            <h1 style="margin: 0; font-size: 1.85rem; color: #0f172a;">🤖 Autonomous Robot Navigation Simulator</h1>
            <p style="margin: 4px 0 0 0; color: #64748b; font-size: 0.95rem;">
                Comparative pathfinding analysis, step-by-step search visualization, and dynamic obstacle replanning for A*, Dijkstra, and Greedy Best-First Search.
            </p>
        </div>
    </div>
    """,
    unsafe_allow_html=True,
)


# --- Sidebar: Configuration & Tools ---
with st.sidebar:
    st.header("⚙️ Configuration & Controls")

    # Algorithm Selection
    algo_option = st.selectbox(
        "Select Active Algorithm",
        ("A* Search", "Dijkstra's Algorithm", "Greedy Best-First Search"),
        index=0,
        help="Choose the algorithm used for single-run search visualization and robot navigation.",
    )
    algo_map = {
        "A* Search": "astar",
        "Dijkstra's Algorithm": "dijkstra",
        "Greedy Best-First Search": "greedy",
    }
    sim.algorithm_name = algo_map[algo_option]

    st.divider()

    # Preset Layouts
    st.subheader("🏢 Warehouse Preset Layouts")
    preset_choice = st.selectbox(
        "Choose Preset Scenario",
        (
            "Amazon Sortation Hub",
            "Terrain Dilemma (Greedy vs Optimal)",
            "Bottleneck Chokepoint",
            "Standard Distribution Center",
        ),
        index=0,
    )
    if st.button("Load Preset Layout", use_container_width=True):
        preset_key_map = {
            "Amazon Sortation Hub": "amazon_sortation",
            "Terrain Dilemma (Greedy vs Optimal)": "terrain_dilemma",
            "Bottleneck Chokepoint": "bottleneck_chokepoint",
            "Standard Distribution Center": "default",
        }
        grid.load_preset(preset_key_map[preset_choice])
        sim.reset()
        st.session_state.search_result = None
        st.session_state.animation_step = 0
        st.success(f"Loaded '{preset_choice}'!")
        st.rerun()

    # Random Obstacle Generator
    with st.expander("🎲 Random Obstacle Generator"):
        density = st.slider("Obstacle Density", 0.05, 0.40, 0.20, step=0.05)
        seed = st.number_input("Random Seed", value=42, min_value=0, max_value=99999)
        if st.button("Generate Random Layout", use_container_width=True):
            grid.generate_random_scenario(obstacle_density=density, seed=int(seed))
            sim.reset()
            st.session_state.search_result = None
            st.session_state.animation_step = 0
            st.success("Generated randomized obstacle scenario.")
            st.rerun()

    st.divider()

    # Grid Dimensions Editor
    with st.expander("📐 Grid Dimensions"):
        col1, col2 = st.columns(2)
        with col1:
            new_rows = st.number_input("Rows", min_value=5, max_value=50, value=grid.rows)
        with col2:
            new_cols = st.number_input("Columns", min_value=5, max_value=60, value=grid.cols)

        if st.button("Resize Grid", use_container_width=True):
            is_valid, msg = validate_grid_dimensions(int(new_rows), int(new_cols))
            if is_valid:
                st.session_state.grid = WarehouseGrid(rows=int(new_rows), cols=int(new_cols))
                st.session_state.sim_engine = SimulationEngine(
                    st.session_state.grid, algorithm_name=sim.algorithm_name
                )
                st.session_state.search_result = None
                st.session_state.animation_step = 0
                st.success(f"Resized grid to {new_rows}x{new_cols}")
                st.rerun()
            else:
                st.error(msg)

    # Terrain Costs Configuration
    with st.expander("⚡ Terrain Cost Weights"):
        st.caption("Customize positive movement costs per cell type:")
        low_c = st.number_input("Express Lane Cost", min_value=0.1, max_value=5.0, value=0.5, step=0.1)
        std_c = st.number_input("Standard Floor Cost", min_value=0.5, max_value=10.0, value=1.0, step=0.5)
        high_c = st.number_input("Rough / Heavy Floor", min_value=1.0, max_value=20.0, value=5.0, step=1.0)
        vhigh_c = st.number_input("Spill / Staging Cost", min_value=2.0, max_value=50.0, value=10.0, step=1.0)
        restr_blocked = st.checkbox("Restricted Zones are Impassable", value=True)

        if st.button("Apply Terrain Costs", use_container_width=True):
            grid.terrain_config = TerrainConfig(
                low_cost=float(low_c),
                standard_cost=float(std_c),
                high_cost=float(high_c),
                very_high_cost=float(vhigh_c),
                restricted_is_blocked=restr_blocked,
            )
            st.success("Updated terrain cost weights.")
            st.rerun()

    # Manual Coordinate Editor
    with st.expander("✏️ Coordinate Editor"):
        edit_tool = st.selectbox(
            "Tool Action",
            (
                "Add Wall / Shelf",
                "Remove Wall (Empty)",
                "Paint Express Lane (0.5)",
                "Paint Rough Floor (5.0)",
                "Paint Spill Zone (10.0)",
                "Set Restricted Zone",
                "Move Start Position",
                "Move Goal Position",
            ),
        )
        ec1, ec2 = st.columns(2)
        with ec1:
            target_r = st.number_input("Target Row", 0, grid.rows - 1, 0)
        with ec2:
            target_c = st.number_input("Target Col", 0, grid.cols - 1, 0)

        if st.button("Apply Cell Edit", use_container_width=True):
            r, c = int(target_r), int(target_c)
            if edit_tool == "Add Wall / Shelf":
                grid.set_cell_type(r, c, CellType.WALL)
            elif edit_tool == "Remove Wall (Empty)":
                grid.set_cell_type(r, c, CellType.EMPTY)
            elif edit_tool == "Paint Express Lane (0.5)":
                grid.set_cell_type(r, c, CellType.TERRAIN_LOW)
            elif edit_tool == "Paint Rough Floor (5.0)":
                grid.set_cell_type(r, c, CellType.TERRAIN_HIGH)
            elif edit_tool == "Paint Spill Zone (10.0)":
                grid.set_cell_type(r, c, CellType.TERRAIN_VERY_HIGH)
            elif edit_tool == "Set Restricted Zone":
                grid.set_cell_type(r, c, CellType.RESTRICTED)
            elif edit_tool == "Move Start Position":
                if not grid.set_start(r, c):
                    st.error("Cannot place start on wall or goal.")
            elif edit_tool == "Move Goal Position":
                if not grid.set_goal(r, c):
                    st.error("Cannot place goal on wall or start.")

            sim.reset()
            st.session_state.search_result = None
            st.rerun()


# --- Main Application Tabs ---
tab1, tab2, tab3 = st.tabs(
    [
        "🗺️ Warehouse Simulation & Navigation",
        "📊 3-Way Benchmark & Comparison",
        "📚 Algorithm Theory & Educational Guide",
    ]
)


# =========================================================================
# TAB 1: Warehouse Simulation & Navigation
# =========================================================================
with tab1:
    # Primary Simulation Controls Bar
    ctrl_col1, ctrl_col2, ctrl_col3, ctrl_col4, ctrl_col5 = st.columns([1.5, 1.5, 1.2, 1.2, 1.5])

    with ctrl_col1:
        if st.button("🔍 Plan Route", type="primary", use_container_width=True):
            success, msg = sim.plan_initial_route()
            st.session_state.search_result = sim.last_search_result
            if success and sim.last_search_result:
                st.session_state.animation_step = len(sim.last_search_result.search_events)
                st.success(msg)
            else:
                st.error(msg)
            st.rerun()

    with ctrl_col2:
        if st.button("▶️ Step Robot Forward", use_container_width=True):
            if sim.robot.status in (RobotStatus.IDLE, RobotStatus.NO_PATH_FOUND):
                sim.plan_initial_route()
            status, completed, err = sim.step()
            if completed:
                st.balloons()
                st.success("Robot successfully reached destination goal!")
            elif err:
                st.warning(err)
            st.rerun()

    with ctrl_col3:
        if st.button("⏩ Auto-Drive Robot", use_container_width=True):
            if sim.robot.status in (RobotStatus.IDLE, RobotStatus.NO_PATH_FOUND):
                sim.plan_initial_route()
            # Advance up to goal or blockage
            steps_taken = 0
            while sim.robot.status == RobotStatus.MOVING or sim.robot.status == RobotStatus.PATH_FOUND:
                status, completed, err = sim.step()
                steps_taken += 1
                if completed or err or steps_taken > 200:
                    break
            if sim.robot.status == RobotStatus.COMPLETED:
                st.balloons()
                st.success(f"Robot arrived at destination in {steps_taken} steps!")
            st.rerun()

    with ctrl_col4:
        if st.button("🔄 Reset Sim", use_container_width=True):
            sim.reset()
            grid.clear_dynamic_obstacles()
            st.session_state.search_result = None
            st.session_state.animation_step = 0
            st.rerun()

    with ctrl_col5:
        # Quick Dynamic Obstacle Placement
        if st.button("💥 Drop Dynamic Obstacle", use_container_width=True):
            if len(sim.planned_path) > sim.robot.step_index + 2:
                # Target a cell 2 steps ahead of the robot
                block_coord = sim.planned_path[sim.robot.step_index + 2]
                inserted, replanned, msg = sim.insert_dynamic_obstacle(block_coord[0], block_coord[1])
                if replanned:
                    st.warning(f"Obstacle dropped at {block_coord}! {msg}")
                else:
                    st.info(msg)
                st.rerun()
            else:
                st.info("Plan route and advance robot first to drop obstacles ahead of the robot.")

    # Status Telemetry HUD Cards
    stat_col1, stat_col2, stat_col3, stat_col4, stat_col5 = st.columns(5)
    with stat_col1:
        st.metric("Simulation Status", sim.robot.status.value)
    with stat_col2:
        st.metric("Robot Location", f"({sim.robot.position[0]}, {sim.robot.position[1]})")
    with stat_col3:
        total_steps = len(sim.planned_path) - 1 if len(sim.planned_path) > 0 else 0
        curr_step = sim.robot.step_index
        st.metric("Progress (Steps)", f"{curr_step} / {total_steps}")
    with stat_col4:
        # Compute traversed cost
        trav_cost = 0.0
        for i in range(1, len(sim.robot.traversed_path)):
            trav_cost += grid.get_cell_cost(*sim.robot.traversed_path[i])
        st.metric("Traversed Cost", f"{trav_cost:.2f}")
    with stat_col5:
        st.metric("Replans Triggered", f"{sim.robot.replan_count} ({round(sim.robot.total_replan_time_s*1000, 1)}ms)")

    # Compute visualization elements
    visited_nodes: list[Coordinate] = []
    frontier_nodes: list[Coordinate] = []

    res = st.session_state.search_result
    if res and res.search_events:
        max_events = len(res.search_events)
        # Search exploration scrubber slider
        scrub_col1, scrub_col2 = st.columns([4, 1])
        with scrub_col1:
            st.session_state.animation_step = st.slider(
                "Search Exploration Timeline Scrubber (Step through actual priority queue events):",
                min_value=0,
                max_value=max_events,
                value=min(st.session_state.animation_step, max_events),
                step=1,
            )
        with scrub_col2:
            st.caption(f"Event: {st.session_state.animation_step} / {max_events}")

        visited_nodes, frontier_nodes = get_search_state_at_step(
            res.search_events, st.session_state.animation_step
        )
    elif sim.robot.status in (RobotStatus.MOVING, RobotStatus.COMPLETED):
        if sim.last_search_result:
            visited_nodes = sim.last_search_result.visited_nodes
            frontier_nodes = sim.last_search_result.frontier_nodes

    # Interactive Plotly Grid Canvas
    fig_grid = render_grid_plotly(
        grid=grid,
        path=sim.planned_path if (sim.robot.status != RobotStatus.IDLE and len(sim.planned_path) > 0) else None,
        visited=visited_nodes,
        frontier=frontier_nodes,
        robot=sim.robot if sim.robot.status != RobotStatus.IDLE else None,
        traversed_path=sim.robot.traversed_path if len(sim.robot.traversed_path) > 1 else None,
        title=f"Warehouse Floor: {algo_option} | Status: {sim.robot.status.value}",
    )
    st.plotly_chart(fig_grid, use_container_width=True)

    # Search Metrics Table (if computed)
    if res and res.found:
        st.markdown("### 📈 Search Exploration Metrics")
        m_c1, m_c2, m_c3, m_c4, m_c5 = st.columns(5)
        m_c1.metric("Optimal Path Found", "Yes" if res.found else "No")
        m_c2.metric("Path Length", f"{res.path_length} cells")
        m_c3.metric("Total Traversal Cost", f"{res.total_cost:.2f}")
        m_c4.metric("Nodes Expanded", f"{res.expanded_nodes}")
        m_c5.metric("Execution Time", f"{res.execution_time_ms:.2f} ms")


# =========================================================================
# TAB 2: 3-Way Benchmark & Comparison
# =========================================================================
with tab2:
    st.markdown("### ⚖️ Controlled Multi-Algorithm Benchmark")
    st.write(
        "Runs **A\* Search**, **Dijkstra's Algorithm**, and **Greedy Best-First Search** "
        "on an **identical immutable snapshot** of the current warehouse grid."
    )

    if st.button("🚀 Run Controlled 3-Way Comparison", type="primary"):
        with st.spinner("Executing algorithms on isolated environment snapshots..."):
            results_dict, comp_df = run_controlled_comparison(grid, "Current Warehouse Map")
            st.session_state.benchmark_df = comp_df
        st.success("Comparison completed!")

    if st.session_state.benchmark_df is not None:
        comp_df = st.session_state.benchmark_df

        st.markdown("#### 📋 Performance Results Table")
        st.dataframe(comp_df, use_container_width=True)

        # Plotly Subplot Comparison Charts
        st.markdown("#### 📊 Comparative Analytics")
        comp_fig = create_comparison_charts(comp_df)
        st.plotly_chart(comp_fig, use_container_width=True)

        # CSV Download Button
        csv_data = comp_df.to_csv(index=False)
        st.download_button(
            label="📥 Download Single-Run Benchmark (CSV)",
            data=csv_data,
            file_name="warehouse_pathfinding_benchmark.csv",
            mime="text/csv",
        )

    st.divider()

    # Repeated Multi-Scenario Experiments
    st.markdown("### 🔬 Automated Batch Experiments across Multiple Random Seeds")
    st.caption("Perform rigorous academic statistical evaluations across multiple reproducible warehouse maps.")

    bc1, bc2, bc3, bc4 = st.columns(4)
    with bc1:
        num_scenarios = st.slider("Scenarios Count", min_value=1, max_value=20, value=5)
    with bc2:
        batch_density = st.slider("Obstacle Density", 0.05, 0.40, 0.20, step=0.05, key="b_dens")
    with bc3:
        batch_rows = st.number_input("Grid Rows", 8, 40, 16, key="b_rows")
    with bc4:
        batch_seed = st.number_input("Base Random Seed", 1, 99999, 100, key="b_seed")

    if st.button("🧪 Execute Multi-Scenario Experiment"):
        with st.spinner(f"Running {num_scenarios} randomized scenarios across 3 algorithms ({num_scenarios * 3} runs)..."):
            detail_df, summary_df = run_multi_scenario_experiment(
                num_scenarios=num_scenarios,
                rows=int(batch_rows),
                cols=int(batch_rows * 1.5),
                obstacle_density=batch_density,
                base_seed=int(batch_seed),
            )
            st.session_state.batch_detail_df = detail_df
            st.session_state.batch_summary_df = summary_df
        st.success("Batch experiments completed!")

    if st.session_state.batch_summary_df is not None:
        st.markdown("#### 📊 Aggregated Statistical Summary")
        st.dataframe(st.session_state.batch_summary_df, use_container_width=True)

        with st.expander("🔍 View Raw Per-Scenario Data"):
            st.dataframe(st.session_state.batch_detail_df, use_container_width=True)

        batch_csv = st.session_state.batch_detail_df.to_csv(index=False)
        st.download_button(
            label="📥 Download Multi-Scenario Dataset (CSV)",
            data=batch_csv,
            file_name=f"multi_scenario_experiments_{num_scenarios}_runs.csv",
            mime="text/csv",
        )


# =========================================================================
# TAB 3: Algorithm Theory & Educational Guide
# =========================================================================
with tab3:
    st.markdown("### 🎓 Pedagogical Theory & Algorithmic Analysis")

    st.markdown(
        """
        #### 1. A* Search
        - **Evaluation Function:** $f(n) = g(n) + h(n)$
        - **$g(n)$:** True accumulated traversal cost from start to $n$.
        - **$h(n)$:** Admissible heuristic estimate to the goal. We employ **scaled Manhattan distance**:
          $$h(n) = (|r_n - r_{\\text{goal}}| + |c_n - c_{\\text{goal}}|) \\times c_{\\min}$$
          where $c_{\\min}$ is the minimum traversal cost across all passable terrain types. This preserves mathematical admissibility: $h(n) \\le h^*(n)$ for all $n$.
        - **Optimality:** Guaranteed optimal under admissible and consistent heuristics.
        - **Time Complexity:** $O((|V| + |E|) \\log |V|)$ in the worst case with binary heap priority queues.
        """
    )

    st.markdown(
        """
        #### 2. Dijkstra's Algorithm
        - **Evaluation Function:** $f(n) = g(n)$ (equivalent to A* with $h(n) = 0$).
        - **Search Strategy:** Uniform-cost exploration in concentric cost contours.
        - **Optimality:** Guaranteed optimal for all graphs with nonnegative edge costs.
        - **Role in Simulator:** Serves as the optimal ground-truth benchmark.
        """
    )

    st.markdown(
        """
        #### 3. Greedy Best-First Search
        - **Evaluation Function:** $\\text{priority}(n) = h(n)$
        - **Search Strategy:** Follows the steepest gradient toward the goal coordinates, completely ignoring accumulated traversal cost $g(n)$.
        - **Suboptimality Demonstration:** On weighted terrain (such as the *Terrain Dilemma* preset), Greedy rushes blindly through expensive mud/spills (cost = 10.0) simply because those cells are geometrically closer to the goal, leading to path costs significantly higher than the optimal paths computed by Dijkstra or A*.
        """
    )

    st.markdown(
        """
        #### 4. Theoretical Comparison Matrix

        | Feature | Dijkstra's Algorithm | A* Search | Greedy Best-First Search |
        | :--- | :--- | :--- | :--- |
        | **Priority Function** | $g(n)$ | $g(n) + h(n)$ | $h(n)$ |
        | **Cost Optimality** | **Guaranteed Optimal** | **Guaranteed Optimal** | **Not Guaranteed** |
        | **Heuristic Used** | None ($h=0$) | Scaled Manhattan ($c_{\\min}$) | Manhattan |
        | **Explored Volume** | Broad (all directions) | Narrow (directed toward goal) | Highly focused (or stuck in traps) |
        | **Terrain Sensitivity**| Fully optimal | Fully optimal | Ignores terrain costs in priority |
        """
    )
