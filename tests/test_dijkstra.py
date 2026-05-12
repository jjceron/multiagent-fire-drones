"""Tests for the Dijkstra pathfinding algorithm."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from firedrones.environment.grid import Grid
from firedrones.environment.cell import CellType
from firedrones.planning.dijkstra import DijkstraPlanner
from firedrones.planning.astar import AStarPlanner


class TestDijkstraBasic:
    def test_finds_path_open_grid(self, small_grid):
        """Dijkstra must find a path on an open grid."""
        planner = DijkstraPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        assert len(path) > 0

    def test_path_ends_at_goal(self, small_grid):
        planner = DijkstraPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        assert path[-1] == (9, 9)

    def test_no_path_when_blocked(self, fully_blocked_grid):
        planner = DijkstraPlanner(fully_blocked_grid)
        path = planner.find_path((0, 0), (5, 5))
        assert path == []

    def test_same_length_as_astar(self, small_grid):
        """
        On a uniform-cost grid, Dijkstra and A* must produce
        paths of equal length (both are optimal).
        """
        astar = AStarPlanner(small_grid)
        dijkstra = DijkstraPlanner(small_grid)
        pa = astar.find_path((0, 0), (9, 9))
        pd = dijkstra.find_path((0, 0), (9, 9))
        assert len(pa) == len(pd), (
            f"Path lengths differ: A*={len(pa)}, Dijkstra={len(pd)}"
        )

    def test_avoids_obstacles(self, grid_with_wall):
        planner = DijkstraPlanner(grid_with_wall)
        path = planner.find_path((0, 0), (9, 0))
        for col, row in path:
            cell = grid_with_wall.get_cell(col, row)
            assert cell.cell_type != CellType.OBSTACLE
