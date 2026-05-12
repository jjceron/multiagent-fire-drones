"""
Region module — groups of adjacent cells sharing a common property.
"""
from __future__ import annotations
from typing import Iterator
from firedrones.environment.cell import Cell, CellType


class Region:
    """
    A named collection of adjacent cells with a shared property.

    Regions can represent, for example, a cluster of fire cells,
    a restricted zone, or a base area.
    """

    def __init__(self, name: str, cells: list[Cell] | None = None) -> None:
        self.name = name
        self._cells: list[Cell] = cells or []

    def add_cell(self, cell: Cell) -> None:
        """Add a cell to this region if not already present."""
        if cell not in self._cells:
            self._cells.append(cell)

    def remove_cell(self, cell: Cell) -> None:
        """Remove a cell from the region."""
        self._cells = [c for c in self._cells if c != cell]

    def contains(self, col: int, row: int) -> bool:
        """Check whether position (col, row) is in this region."""
        return any(c.col == col and c.row == row for c in self._cells)

    def cells_of_type(self, cell_type: CellType) -> list[Cell]:
        """Return all cells matching a given CellType."""
        return [c for c in self._cells if c.cell_type == cell_type]

    def __iter__(self) -> Iterator[Cell]:
        return iter(self._cells)

    def __len__(self) -> int:
        return len(self._cells)

    def __repr__(self) -> str:
        return f"Region(name={self.name!r}, size={len(self._cells)})"
