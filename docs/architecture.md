# Architecture Documentation: Autonomous Robot Navigation Simulator

## 1. System Overview

The application follows a clean, decoupled modular architecture where the **Environment Model**, **Pathfinding Algorithms**, **Simulation Engine**, **Dynamic Replanning**, **Benchmarking Runner**, and **Visualization Layer** communicate through clear, strongly-typed interfaces.

```
                      +-----------------------------+
                      |   Interactive UI (Streamlit)|
                      |           app.py            |
                      +--------------+--------------+
                                     |
               +---------------------+---------------------+
               |                     |                     |
               v                     v                     v
     +-------------------+ +-------------------+ +-------------------+
     |   Visualization   | | Simulation Engine | |   Benchmarking    |
     |   grid_renderer   | |    simulator.py   | |     runner.py     |
     |     charts.py     | |   replanning.py   | |    metrics.py     |
     |   animation.py    | |     robot.py      | +---------+---------+
     +-------------------+ +---------+---------+           |
                                     |                     |
                                     +----------+----------+
                                                |
                                                v
                                    +-----------------------+
                                    | Pathfinding Algorithms|
                                    |        astar.py       |
                                    |      dijkstra.py      |
                                    | greedy_best_first.py  |
                                    |     heuristics.py     |
                                    +-----------+-----------+
                                                |
                                                v
                                    +-----------------------+
                                    |   Warehouse Grid      |
                                    |        grid.py        |
                                    |        cell.py        |
                                    |       terrain.py      |
                                    |      obstacles.py     |
                                    +-----------------------+
```

---

## 2. Core Subsystems

### 2.1 Environment Subsystem (`src/environment/`)
- `cell.py`: Defines the `CellType` enumeration (`EMPTY`, `WALL`, `RESTRICTED`, `TERRAIN_LOW`, `TERRAIN_STANDARD`, `TERRAIN_HIGH`, `TERRAIN_VERY_HIGH`, `DYNAMIC_OBSTACLE`), coordinate types, and per-cell state.
- `terrain.py`: Defines the `TerrainConfig` class, specifying positive traversal costs for each cell type and calculating the minimum permitted movement cost across passable terrain.
- `obstacles.py`: Handles deterministic pseudo-random wall generation parameterized by density and random seed, ensuring safe clearances around start and goal.
- `grid.py`: The `WarehouseGrid` class encapsulates the 2D matrix, enforces bounds, handles orthogonal neighbor generation (North, South, East, West), prevents diagonal traversal, and provides immutable cloning via `clone()`.

### 2.2 Algorithm Subsystem (`src/algorithms/`)
- `common.py`: Standardizes the `SearchResult` and `SearchEvent` contracts. Every algorithm returns:
  - `found`: boolean
  - `path`: ordered list of coordinates
  - `expanded_nodes`: count of nodes popped from priority queue
  - `discovered_nodes`: unique coordinates added to frontier
  - `generated_nodes`: total candidate successor states examined
  - `total_cost`: exact path traversal cost under destination-cell convention
  - `path_length`: number of steps
  - `execution_time`: high-resolution computation duration (seconds)
  - `search_events`: chronologically ordered timeline of search state changes
- `heuristics.py`: Admissible Manhattan distance and scaled Manhattan distance functions.
- `astar.py`, `dijkstra.py`, `greedy_best_first.py`: Independent, from-scratch implementations. No third-party pathfinding libraries are used.

### 2.3 Simulation & Replanning Subsystem (`src/simulation/`)
- `robot.py`: Models the mobile robot agent (`Robot`), tracking position, orientation (`Direction`), step progress, traversed trajectory history, and status flags (`IDLE`, `SEARCHING`, `MOVING`, `REPLANNING`, `COMPLETED`, `NO_PATH_FOUND`).
- `replanning.py`: Detects when dynamic obstacles compromise the remaining planned trajectory. Rather than resetting the robot, it freezes the robot's physical position, treats the robot's current coordinates as the start node, and executes a targeted replan to the original goal.
- `simulator.py`: `SimulationEngine` ties the grid, robot, and search algorithms together into an event-driven discrete simulation.

### 2.4 Benchmarking Subsystem (`src/benchmarking/`)
- `runner.py`: Provides `run_controlled_comparison` which gives each algorithm an isolated, identical clone snapshot of the warehouse grid.
- Also implements `run_multi_scenario_experiment` to automate batch statistical experiments across varying obstacle densities, terrain configurations, and seeds, outputting structured Pandas DataFrames.
- `metrics.py`: Defines standardized benchmark record models.

### 2.5 Visualization Subsystem (`src/visualization/`)
- `grid_renderer.py`: Uses Plotly to build interactive warehouse visualizations with distinct color layers for terrain, walls, restricted zones, explored nodes, frontier, planned paths, traversed trails, and the robot agent.
- `charts.py`: Generates comparative 4-panel subplots (cost, expanded nodes, length, execution time) for quick visual assessment.
- `animation.py`: Maps recorded `SearchEvent` streams into visual exploration states for step-by-step playback.

---

## 3. Data Flow & Invariant Guarantees

1. **Snapshot Isolation:** Benchmarking runs never mutate the user's active grid. Each algorithm receives `grid.clone()`.
2. **Deterministic Reproducibility:** Generating maps with the same random seed produces bitwise identical obstacle layouts.
3. **No Retroactive Robot Teleportation:** Dynamic obstacle insertion guarantees that the robot remains on its current valid cell; replanning always originates from the robot's current coordinates.
4. **Admissibility Scaling:** Nonuniform terrain costs dynamically scale the A* Manhattan heuristic by $c_{\min}$, guaranteeing that A* and Dijkstra always compute identical optimal traversal costs.
