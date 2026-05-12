"""Tests for the A* pathfinding algorithm."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from firedrones.environment.grid import Grid
from firedrones.environment.cell import CellType
from firedrones.planning.astar import AStarPlanner


class TestAStarBasic:
    def test_finds_path_open_grid(self, small_grid):
        """A* must find a path on an open grid."""
        planner = AStarPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        assert len(path) > 0, "A* should find a path on an open grid"

    def test_path_ends_at_goal(self, small_grid):
        """The last element of the path must be the goal."""
        planner = AStarPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        assert path[-1] == (9, 9)

    def test_path_starts_adjacent_to_start(self, small_grid):
        """The first element must be a neighbour of start (not start itself)."""
        planner = AStarPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        sc, sr = 0, 0
        fc, fr = path[0]
        assert abs(sc - fc) + abs(sr - fr) == 1

    def test_path_length_minimum(self, small_grid):
        """Path length should be at least Manhattan distance from start to goal."""
        planner = AStarPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        assert len(path) >= 18  # Manhattan = 9+9

    def test_same_start_and_goal_returns_empty(self, small_grid):
        """If start equals goal, the path should be empty."""
        planner = AStarPlanner(small_grid)
        path = planner.find_path((3, 3), (3, 3))
        assert path == []


class TestAStarObstacles:
    def test_avoids_obstacles(self, grid_with_wall):
        """A* path must not pass through obstacle cells."""
        planner = AStarPlanner(grid_with_wall)
        path = planner.find_path((0, 0), (9, 0))
        assert len(path) > 0, "A path should exist going around the wall"
        for col, row in path:
            cell = grid_with_wall.get_cell(col, row)
            assert cell.cell_type != CellType.OBSTACLE, \
                f"Path passed through obstacle at ({col},{row})"

    def test_no_path_when_blocked(self, fully_blocked_grid):
        """A* must return empty list when goal is completely blocked."""
        planner = AStarPlanner(fully_blocked_grid)
        path = planner.find_path((0, 0), (5, 5))
        assert path == [], "Should return empty path when goal is unreachable"

    def test_out_of_bounds_goal_returns_empty(self, small_grid):
        """A* must return empty list for out-of-bounds goal."""
        planner = AStarPlanner(small_grid)
        path = planner.find_path((0, 0), (20, 20))
        assert path == []

    def test_obstacle_goal_returns_empty(self, small_grid):
        """A* must return empty list if the goal cell is an obstacle."""
        small_grid.set_type(9, 9, CellType.OBSTACLE)
        planner = AStarPlanner(small_grid)
        path = planner.find_path((0, 0), (9, 9))
        assert path == []
