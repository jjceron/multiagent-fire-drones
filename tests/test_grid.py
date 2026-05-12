"""Tests for the Grid class."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from firedrones.environment.grid import Grid
from firedrones.environment.cell import CellType


class TestGridBasic:
    def test_dimensions(self):
        g = Grid(15, 10)
        assert g.cols == 15
        assert g.rows == 10

    def test_invalid_dimensions(self):
        with contextlib.suppress(ValueError):
            Grid(0, 10)
        with contextlib.suppress(ValueError):
            Grid(10, 0)

    def test_in_bounds(self):
        g = Grid(10, 10)
        assert g.in_bounds(0, 0)
        assert g.in_bounds(9, 9)
        assert not g.in_bounds(10, 0)
        assert not g.in_bounds(0, 10)
        assert not g.in_bounds(-1, 0)

    def test_get_cell_returns_none_out_of_bounds(self):
        g = Grid(10, 10)
        assert g.get_cell(10, 10) is None
        assert g.get_cell(-1, 0) is None

    def test_all_cells_start_empty(self):
        g = Grid(5, 5)
        for cell in g.all_cells():
            assert cell.cell_type == CellType.EMPTY

    def test_set_type(self):
        g = Grid(10, 10)
        g.set_type(3, 4, CellType.OBSTACLE)
        assert g.get_cell(3, 4).cell_type == CellType.OBSTACLE

    def test_is_traversable_obstacle(self):
        g = Grid(10, 10)
        g.set_type(5, 5, CellType.OBSTACLE)
        assert not g.is_traversable(5, 5)

    def test_is_traversable_empty(self):
        g = Grid(10, 10)
        assert g.is_traversable(5, 5)

    def test_neighbours_count_center(self):
        g = Grid(10, 10)
        # Center cell: 4 neighbours
        neighbours = g.neighbours(5, 5)
        assert len(neighbours) == 4

    def test_neighbours_count_corner(self):
        g = Grid(10, 10)
        # Corner cell: 2 neighbours
        neighbours = g.neighbours(0, 0)
        assert len(neighbours) == 2

    def test_neighbours_exclude_obstacles(self):
        g = Grid(10, 10)
        g.set_type(4, 5, CellType.OBSTACLE)
        neighbours = g.neighbours(5, 5)
        positions = [n.position for n in neighbours]
        assert (4, 5) not in positions

    def test_cells_of_type(self):
        g = Grid(10, 10)
        g.set_type(1, 1, CellType.OBSTACLE)
        g.set_type(2, 2, CellType.OBSTACLE)
        obs = g.cells_of_type(CellType.OBSTACLE)
        assert len(obs) == 2

    def test_place_obstacles(self):
        g = Grid(10, 10)
        positions = [(1, 1), (2, 2), (3, 3)]
        g.place_obstacles(positions)
        for col, row in positions:
            assert g.get_cell(col, row).cell_type == CellType.OBSTACLE

    def test_reset(self):
        g = Grid(10, 10)
        g.set_type(3, 3, CellType.OBSTACLE)
        g.set_type(4, 4, CellType.FIRE)
        g.reset()
        for cell in g.all_cells():
            assert cell.cell_type == CellType.EMPTY
