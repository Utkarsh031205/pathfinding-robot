# Autonomous Robot Navigation Simulator

An interactive, academically defensible educational simulation application designed to demonstrate, analyze, and benchmark classical graph pathfinding algorithms—**A\* Search**, **Dijkstra's Algorithm**, and **Greedy Best-First Search**—operating in a 2D industrial warehouse environment.

The simulator features non-uniform weighted terrain, safety zones, step-by-step priority queue exploration visualization, animated robot motion, runtime dynamic obstacle insertion, and automatic route replanning.

---

## 1. Project Overview & Problem Statement

In automated warehousing and industrial logistics (such as Amazon Fulfillment Centers and automated sortation hubs), autonomous mobile robots (AMRs) must navigate complex floors characterized by:
- Narrow aisles and tall storage racks (impassable walls).
- Specialized floor zones (high-speed express conveyor lanes vs. rough concrete or congested staging areas).
- Dynamic unexpected events (fallen pallets, pedestrian traffic, or spill blockages).

Finding the optimal path is non-trivial: the shortest geometric distance is not necessarily the cheapest route when crossing expensive terrain. Furthermore, when unexpected blockages appear ahead of a navigating robot, the robot must safely freeze before collision and replan a detour from its current physical position.

This project implements a fully functional, modular simulator built from scratch to study these algorithmic trade-offs.

---

## 2. Key Features

1. **Interactive 2D Warehouse Grid:**
   - 4-directional orthogonal movement (Up, Down, Left, Right; no diagonal traversal).
   - Configurable grid dimensions (5×5 up to 50×60).
   - Rich preset layouts: *Amazon Sortation Hub*, *Terrain Dilemma*, *Bottleneck Chokepoint*, and *Standard Distribution Center*.
   - Reproducible pseudo-random map generation using deterministic seeds.

2. **Three Core Algorithms Implemented from Scratch:**
   - **A\* Search:** $f(n) = g(n) + h(n)$ with scaled Manhattan heuristic to guarantee admissibility over weighted terrain ($h(n) \le h^*(n)$).
   - **Dijkstra's Algorithm:** Uniform-cost exploration with priority queue on $g(n)$, providing the guaranteed optimal-cost ground truth.
   - **Greedy Best-First Search:** Heuristic-driven search prioritizing $h(n)$, demonstrating fast exploration but susceptibility to suboptimal routes on weighted terrain.

3. **Step-by-Step Search Exploration Visualization:**
   - Real recorded priority queue events (`DISCOVER`, `EXPAND`, `UPDATE_G`, `GOAL_SETTLED`).
   - Interactive exploration scrubber slider to step through the chronological search tree expansion.

4. **Animated Robot Navigation Simulation:**
   - Robot agent tracking position, orientation, step progress, traversed cost, and trajectory trail.
   - Discrete step-by-step movement and automated driving modes.

5. **Dynamic Obstacle Insertion & Safe Replanning:**
   - Runtime insertion of obstacles directly ahead of the moving robot.
   - Safe collision detection: robot freezes before entering blocked cell.
   - Targeted replanning originating from the robot's preserved physical position.
   - Multi-replan tracking, latency measurement, and trapped-state handling.

6. **Controlled 3-Way Benchmarking & Analytics Dashboard:**
   - Executes all 3 algorithms on identical immutable snapshots (`grid.clone()`).
   - Comparative multi-metric table: Path Found, Path Length, Total Traversal Cost, Nodes Expanded, Nodes Discovered, Generated Successors, and Execution Time (ms).
   - 4-panel interactive Plotly comparison charts.
   - Multi-scenario statistical batch experiment runner with mean/median calculation and 1-click CSV export.

7. **Pedagogical Reference Guide:**
   - Complete theory, admissibility proofs, consistency, and time/space complexity analysis ($O((|V| + |E|) \log |V|)$).

---

## 3. Technology Stack

- **Python 3.10+ / 3.12**
- **Streamlit** (Interactive web application framework)
- **NumPy & Pandas** (Grid matrices, structured benchmark tables, and CSV exports)
- **Plotly** (High-resolution interactive grid visualizations and comparative bar charts)
- **pytest** (Comprehensive automated unit test suite)

---

## 4. Project Structure

```
autonomous-robot-navigation/
├── app.py                     # Main Streamlit application entry point
├── requirements.txt           # Python dependency manifest
├── README.md                  # Comprehensive project documentation
├── .gitignore                 # Version control ignore rules
├── .streamlit/
│   └── config.toml            # Server port, headless, and theme settings
├── src/
│   ├── algorithms/            # Pure pathfinding algorithm implementations
│   │   ├── __init__.py
│   │   ├── astar.py           # A* with scaled Manhattan heuristic
│   │   ├── dijkstra.py        # Dijkstra optimal uniform-cost baseline
│   │   ├── greedy_best_first.py # Greedy Best-First Search
│   │   ├── heuristics.py      # Manhattan, scaled Manhattan, Euclidean
│   │   └── common.py          # SearchResult, SearchEvent, and parent reconstruction
│   ├── environment/           # Warehouse grid and terrain modeling
│   │   ├── __init__.py
│   │   ├── grid.py            # WarehouseGrid class and orthogonal rules
│   │   ├── cell.py            # Cell and CellType enum
│   │   ├── terrain.py         # TerrainConfig and cost models
│   │   └── obstacles.py       # Random wall generation with seed support
│   ├── simulation/            # Robot state and dynamic replanning
│   │   ├── __init__.py
│   │   ├── robot.py           # Robot agent state and orientation
│   │   ├── simulator.py       # SimulationEngine coordinator
│   │   └── replanning.py      # Replan logic preserving robot position
│   ├── visualization/         # Visual rendering components
│   │   ├── __init__.py
│   │   ├── grid_renderer.py   # Plotly grid heatmap and overlay renderer
│   │   ├── animation.py       # Event state reconstruction for scrubber
│   │   └── charts.py          # 2x2 comparison subplots
│   ├── benchmarking/          # Benchmarking and statistical experiments
│   │   ├── __init__.py
│   │   ├── runner.py          # Controlled comparison & batch experiment runner
│   │   └── metrics.py         # Metric data classes
│   └── utils/
│       ├── __init__.py
│       └── validation.py      # Grid dimension and coordinate validation
├── tests/                     # Automated pytest test suite (22 tests)
│   ├── __init__.py
│   ├── test_astar.py          # A* optimality and admissibility tests
│   ├── test_dijkstra.py       # Dijkstra shortest path and edge cases
│   ├── test_greedy.py         # Greedy suboptimality on terrain dilemma
│   ├── test_grid.py           # Grid bounds, costs, 4-directional rules
│   ├── test_replanning.py     # Dynamic obstacle insertion & safe replanning
│   └── test_benchmarking.py   # Snapshot isolation & batch experiments
└── docs/
    ├── algorithms.md          # Theoretical foundations and complexity bounds
    └── architecture.md        # System architecture and data flow diagrams
```

---

## 5. Installation & Setup Instructions

### Prerequisites
- Python 3.10, 3.11, or 3.12 installed.

### Setup Virtual Environment
```bash
# Clone the repository
git clone https://github.com/example/autonomous-robot-navigation.git
cd autonomous-robot-navigation

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install required packages
pip install -r requirements.txt
```

---

## 6. How to Run the Application

Launch the interactive Streamlit simulator:
```bash
streamlit run app.py
```
Open your web browser at `http://localhost:3000` (or the default Streamlit URL indicated in the terminal).

---

## 7. Running the Automated Test Suite

Run the full pytest suite:
```bash
pytest tests/ -v
```
All 22 unit tests verify:
- Grid boundary constraints and strictly orthogonal 4-directional movement (no diagonal traversal).
- Dijkstra optimal uniform-cost shortest paths.
- A* optimality and scaled Manhattan heuristic admissibility.
- Greedy Best-First Search suboptimality on weighted terrain dilemmas.
- Safe dynamic obstacle detection, position preservation, and replanning.
- Benchmarking snapshot isolation and CSV reproduction across random seeds.

---

## 8. Terrain Cost Model

Traversal costs follow the **destination-cell convention**:
$$\text{cost}(u \to v) = c(v)$$

Default weights:
- **Empty / Standard Concrete:** $1.0$
- **Express Conveyor Lane:** $0.5$ (low cost)
- **Rough / Heavy Pallet Staging:** $5.0$ (high cost)
- **Hazardous Spill / Bottleneck:** $10.0$ (very high cost)
- **Restricted Safety Zone:** Impassable by default (or configurable penalty)
- **Storage Rack / Shelf:** Impassable wall ($\infty$)

### Heuristic Admissibility Scaling for A*
To ensure that $h(n) \le h^*(n)$ even when express terrain exists with cost $< 1.0$:
$$h(n) = (|r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|) \times \min(c(v))$$
This guarantees that A* always returns the exact same optimal cost as Dijkstra's algorithm.

---

## 9. Dynamic Obstacle Replanning Protocol

1. Robot receives initial path from start to goal.
2. Robot navigates step-by-step along the path.
3. User introduces a dynamic obstacle ahead of the robot.
4. Simulator detects that the upcoming route is blocked.
5. The robot halts immediately before entering the blocked cell.
6. Replanning is triggered from the robot's current physical position to the destination goal.
7. If an alternative route exists, the robot continues along the updated trajectory.
8. If the goal is blocked or unreachable, the robot safely stops and displays a diagnostic warning.

---

## 10. License

Developed for educational demonstration and computer science benchmarking.
