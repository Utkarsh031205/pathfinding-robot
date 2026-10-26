"""
Two-dimensional warehouse grid environment model.
Enforces 4-directional orthogonal movement, boundary constraints, and cost models.
"""
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
from src.environment.cell import Cell, CellType, Coordinate
from src.environment.terrain import TerrainConfig
from src.environment.obstacles import generate_random_walls


class WarehouseGrid:
    """
    2D Grid representation of a warehouse floor.
    """

    def __init__(
        self,
        rows: int = 20,
        cols: int = 30,
        terrain_config: Optional[TerrainConfig] = None,
        start: Optional[Coordinate] = None,
        goal: Optional[Coordinate] = None,
    ):
        self.rows = max(5, min(50, rows))
        self.cols = max(5, min(60, cols))
        self.terrain_config = terrain_config if terrain_config else TerrainConfig()

        self.start: Coordinate = start if start and self.is_in_bounds(*start) else (1, 1)
        self.goal: Coordinate = (
            goal if goal and self.is_in_bounds(*goal) else (self.rows - 2, self.cols - 2)
        )

        # Internal cell storage mapping (row, col) -> Cell
        self.grid: Dict[Coordinate, Cell] = {}
        self._init_cells()

    def _init_cells(self) -> None:
        for r in range(self.rows):
            for c in range(self.cols):
                self.grid[(r, c)] = Cell(row=r, col=c, cell_type=CellType.EMPTY, cost=self.terrain_config.standard_cost)

    def is_in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def get_cell(self, r: int, c: int) -> Cell:
        if not self.is_in_bounds(r, c):
            raise IndexError(f"Cell ({r}, {c}) is outside grid boundaries ({self.rows}x{self.cols}).")
        return self.grid[(r, c)]

    def set_cell_type(self, r: int, c: int, cell_type: CellType, force: bool = False) -> bool:
        if not self.is_in_bounds(r, c):
            return False

        if not force and ((r, c) == self.start or (r, c) == self.goal):
            if cell_type in (CellType.WALL, CellType.DYNAMIC_OBSTACLE):
                return False

        cell = self.grid[(r, c)]
        cell.cell_type = cell_type
        cell.cost = self.terrain_config.get_cell_cost(cell_type)
        return True

    def set_start(self, r: int, c: int) -> bool:
        if not self.is_in_bounds(r, c) or (r, c) == self.goal:
            return False
        if not self.is_passable(r, c):
            return False
        self.start = (r, c)
        return True

    def set_goal(self, r: int, c: int) -> bool:
        if not self.is_in_bounds(r, c) or (r, c) == self.start:
            return False
        if not self.is_passable(r, c):
            return False
        self.goal = (r, c)
        return True

    def is_passable(self, r: int, c: int) -> bool:
        if not self.is_in_bounds(r, c):
            return False
        cell = self.grid[(r, c)]
        return cell.is_passable(restricted_as_high_cost=not self.terrain_config.restricted_is_blocked)

    def get_cell_cost(self, r: int, c: int) -> float:
        if not self.is_in_bounds(r, c):
            return float("inf")
        return self.terrain_config.get_cell_cost(self.grid[(r, c)].cell_type)

    def get_neighbors(self, r: int, c: int) -> List[Tuple[Coordinate, str, float]]:
        """
        Returns list of valid orthogonal neighbors: [((nr, nc), direction_name, step_cost), ...]
        Strict 4-directional movement: Up, Down, Left, Right.
        No diagonal movements.
        """
        deltas = [
            (-1, 0, "UP"),
            (1, 0, "DOWN"),
            (0, -1, "LEFT"),
            (0, 1, "RIGHT"),
        ]
        neighbors = []
        for dr, dc, direction in deltas:
            nr, nc = r + dr, c + dc
            if self.is_in_bounds(nr, nc) and self.is_passable(nr, nc):
                step_cost = self.get_cell_cost(nr, nc)
                neighbors.append(((nr, nc), direction, step_cost))
        return neighbors

    def clear_all(self) -> None:
        """Resets all cells to EMPTY."""
        for cell in self.grid.values():
            cell.cell_type = CellType.EMPTY
            cell.cost = self.terrain_config.standard_cost

    def clear_dynamic_obstacles(self) -> None:
        """Removes only dynamic obstacles."""
        for cell in self.grid.values():
            if cell.cell_type == CellType.DYNAMIC_OBSTACLE:
                cell.cell_type = CellType.EMPTY
                cell.cost = self.terrain_config.standard_cost

    def load_preset(self, preset_name: str) -> None:
        """Loads realistic industrial warehouse presets."""
        self.clear_all()
        if preset_name == "amazon_sortation":
            self.rows, self.cols = 20, 30
            self._init_cells()
            self.start = (1, 1)
            self.goal = (18, 28)

            # Central express conveyor lane
            for c in range(2, 28):
                self.set_cell_type(10, c, CellType.TERRAIN_LOW)

            # Parallel storage shelf racks with picking openings
            for c in range(4, 26, 4):
                for r in range(2, 9):
                    if r != 5:
                        self.set_cell_type(r, c, CellType.WALL)
                for r in range(12, 19):
                    if r != 15:
                        self.set_cell_type(r, c, CellType.WALL)

            # Heavy staging areas
            for r in range(3, 8):
                self.set_cell_type(r, 2, CellType.TERRAIN_HIGH)
                self.set_cell_type(r, 27, CellType.TERRAIN_HIGH)

            # Restricted safety zone
            self.set_cell_type(6, 14, CellType.RESTRICTED)
            self.set_cell_type(6, 15, CellType.RESTRICTED)
            self.set_cell_type(7, 14, CellType.RESTRICTED)
            self.set_cell_type(7, 15, CellType.RESTRICTED)

        elif preset_name == "terrain_dilemma":
            # Demonstrates suboptimality of Greedy Best-First vs Dijkstra/A*
            self.rows, self.cols = 15, 25
            self._init_cells()
            self.start = (7, 2)
            self.goal = (7, 22)

            # Center direct line has expensive mud/spill
            for r in range(5, 10):
                for c in range(7, 18):
                    self.set_cell_type(r, c, CellType.TERRAIN_VERY_HIGH)

            # Express bypass at top row
            for c in range(3, 22):
                self.set_cell_type(2, c, CellType.TERRAIN_LOW)

        elif preset_name == "bottleneck_chokepoint":
            self.rows, self.cols = 18, 24
            self._init_cells()
            self.start = (9, 2)
            self.goal = (9, 21)

            # Dividing warehouse wall with narrow single door
            for r in range(18):
                if r not in (8, 9):
                    self.set_cell_type(r, 12, CellType.WALL)

            for r in range(6, 12):
                self.set_cell_type(r, 11, CellType.TERRAIN_HIGH)

        else:  # default standard layout
            self.rows, self.cols = 20, 30
            self._init_cells()
            self.start = (2, 2)
            self.goal = (17, 27)

            for r in range(4, 9):
                self.set_cell_type(r, 8, CellType.WALL)
                self.set_cell_type(r, 16, CellType.WALL)
            for r in range(11, 16):
                self.set_cell_type(r, 8, CellType.WALL)
                self.set_cell_type(r, 16, CellType.WALL)
                self.set_cell_type(r, 22, CellType.WALL)

            for c in range(4, 26):
                self.set_cell_type(10, c, CellType.TERRAIN_LOW)

            self.set_cell_type(5, 12, CellType.RESTRICTED)
            self.set_cell_type(5, 13, CellType.RESTRICTED)

    def generate_random_scenario(self, obstacle_density: float = 0.2, seed: int = 42) -> None:
        self.clear_all()
        walls = generate_random_walls(self.rows, self.cols, obstacle_density, seed, self.start, self.goal)
        for r, c in walls:
            self.set_cell_type(r, c, CellType.WALL)

    def clone(self) -> "WarehouseGrid":
        """Returns an isolated, deep clone of the environment."""
        new_grid = WarehouseGrid(
            rows=self.rows,
            cols=self.cols,
            terrain_config=TerrainConfig(
                low_cost=self.terrain_config.low_cost,
                standard_cost=self.terrain_config.standard_cost,
                high_cost=self.terrain_config.high_cost,
                very_high_cost=self.terrain_config.very_high_cost,
                restricted_penalty=self.terrain_config.restricted_penalty,
                restricted_is_blocked=self.terrain_config.restricted_is_blocked,
            ),
            start=self.start,
            goal=self.goal,
        )
        for (r, c), cell in self.grid.items():
            new_grid.set_cell_type(r, c, cell.cell_type, force=True)
        return new_grid

    def to_cost_matrix(self) -> np.ndarray:
        """Returns a 2D NumPy array of movement costs."""
        mat = np.zeros((self.rows, self.cols), dtype=float)
        for (r, c), cell in self.grid.items():
            mat[r, c] = cell.cost
        return mat
