"""
Terrain configuration and cost modeling for the warehouse simulation.
"""
from dataclasses import dataclass
from src.environment.cell import CellType


@dataclass
class TerrainConfig:
    low_cost: float = 0.5            # Express conveyor / smooth path
    standard_cost: float = 1.0       # Standard concrete floor
    high_cost: float = 5.0           # Heavy traffic / rough floor
    very_high_cost: float = 10.0     # Spill / staging bottleneck
    restricted_penalty: float = 25.0 # Penalty if restricted zone is traversable
    restricted_is_blocked: bool = True # Impassable by default

    def get_cell_cost(self, cell_type: CellType) -> float:
        """
        Destination cell traversal cost convention:
        Moving from node u to v costs cost(v).
        """
        if cell_type in (CellType.WALL, CellType.DYNAMIC_OBSTACLE):
            return float("inf")
        if cell_type == CellType.RESTRICTED:
            return float("inf") if self.restricted_is_blocked else self.restricted_penalty
        if cell_type == CellType.TERRAIN_LOW:
            return self.low_cost
        if cell_type in (CellType.TERRAIN_STANDARD, CellType.EMPTY, CellType.START, CellType.GOAL):
            return self.standard_cost
        if cell_type == CellType.TERRAIN_HIGH:
            return self.high_cost
        if cell_type == CellType.TERRAIN_VERY_HIGH:
            return self.very_high_cost
        return self.standard_cost

    def min_permitted_cost(self) -> float:
        """
        Returns the minimum possible positive traversal cost across all passable terrain types.
        Used to scale the Manhattan heuristic for A* to guarantee admissibility:
        h(n) = Manhattan(n, goal) * min_permitted_cost <= true shortest remaining cost.
        """
        return min(self.low_cost, self.standard_cost, self.high_cost, self.very_high_cost)
