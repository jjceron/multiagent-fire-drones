"""
Grid module — 2D rectangular workspace divided into cells.
"""
from __future__ import annotations
import random
from firedrones.environment.cell import Cell, CellType


class Grid:
    """
    A 2D grid of Cell objects representing the simulation workspace.

    The grid uses (col, row) indexing where col is the x-axis and
    row is the y-axis, with (0, 0) at the top-left corner.

    Attributes:
        cols: Number of columns.
        rows: Number of rows.
    """

    def __init__(self, cols: int, rows: int) -> None:
        if cols <= 0 or rows <= 0:
            raise ValueError("Grid dimensions must be positive integers.")
        self.cols = cols
        self.rows = rows
        self._cells: list[list[Cell]] = [
            [Cell(c, r) for r in range(rows)] for c in range(cols)
        ]

    # ------------------------------------------------------------------
    # Cell access
    # ------------------------------------------------------------------

    def get_cell(self, col: int, row: int) -> Cell | None:
        """Return the Cell at (col, row), or None if out of bounds."""
        if self.in_bounds(col, row):
            return self._cells[col][row]
        return None

    def set_type(self, col: int, row: int, cell_type: CellType) -> None:
        """Set the type of the cell at (col, row)."""
        cell = self.get_cell(col, row)
        if cell is not None:
            cell.cell_type = cell_type

    def in_bounds(self, col: int, row: int) -> bool:
        """Return True if (col, row) is within the grid boundaries."""
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_traversable(self, col: int, row: int) -> bool:
        """Return True if the cell at (col, row) can be entered by a drone."""
        cell = self.get_cell(col, row)
        return cell is not None and cell.is_traversable()

    # ------------------------------------------------------------------
    # Neighbours
    # ------------------------------------------------------------------

    def neighbours(self, col: int, row: int) -> list[Cell]:
        """
        Return the 4-connected traversable neighbours of (col, row).
        Diagonals are not included (grid movement is cardinal only).
        """
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        result: list[Cell] = []
        for dc, dr in directions:
            nc, nr = col + dc, row + dr
            if self.in_bounds(nc, nr) and self.is_traversable(nc, nr):
                result.append(self._cells[nc][nr])
        return result

    # ------------------------------------------------------------------
    # Bulk helpers
    # ------------------------------------------------------------------

    def all_cells(self) -> list[Cell]:
        """Return a flat list of every cell in the grid."""
        return [self._cells[c][r] for c in range(self.cols) for r in range(self.rows)]

    def cells_of_type(self, cell_type: CellType) -> list[Cell]:
        """Return all cells matching a given CellType."""
        return [cell for cell in self.all_cells() if cell.cell_type == cell_type]

    def random_empty_cell(self, rng: random.Random | None = None) -> Cell | None:
        """Return a random EMPTY cell, or None if none exist."""
        empties = self.cells_of_type(CellType.EMPTY)
        if not empties:
            return None
        r = rng or random
        return r.choice(empties)

    def place_obstacles(self, positions: list[tuple[int, int]]) -> None:
        """Mark the given positions as OBSTACLE cells."""
        for col, row in positions:
            self.set_type(col, row, CellType.OBSTACLE)

    def clear_obstacles(self) -> None:
        """Remove all obstacles (set them back to EMPTY)."""
        for cell in self.cells_of_type(CellType.OBSTACLE):
            cell.cell_type = CellType.EMPTY

    def place_random_obstacles(
        self, count: int, rng: random.Random | None = None
    ) -> list[tuple[int, int]]:
        """
        Place `count` obstacles at random EMPTY positions.
        Returns the list of placed positions.
        """
        r = rng or random
        placed: list[tuple[int, int]] = []
        empties = self.cells_of_type(CellType.EMPTY)
        r.shuffle(empties)
        for cell in empties[:count]:
            cell.cell_type = CellType.OBSTACLE
            placed.append(cell.position)
        return placed

    def reset(self) -> None:
        """Reset all cells to EMPTY."""
        for cell in self.all_cells():
            cell.cell_type = CellType.EMPTY

    def __repr__(self) -> str:
        return f"Grid(cols={self.cols}, rows={self.rows})"
