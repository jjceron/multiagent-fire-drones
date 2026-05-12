"""
Cell module — defines the atomic unit of the grid workspace.
"""
from __future__ import annotations
from enum import Enum, auto


class CellType(Enum):
    """Enumeration of possible cell types in the grid."""
    EMPTY = auto()
    OBSTACLE = auto()
    BASE = auto()
    FIRE = auto()


class Cell:
    """
    Represents a single cell in the 2D grid.

    Attributes:
        col: Column index (x-axis).
        row: Row index (y-axis).
        cell_type: The current type of this cell.
    """

    __slots__ = ("col", "row", "cell_type")

    def __init__(self, col: int, row: int, cell_type: CellType = CellType.EMPTY) -> None:
        self.col = col
        self.row = row
        self.cell_type = cell_type

    @property
    def position(self) -> tuple[int, int]:
        """Return (col, row) tuple."""
        return (self.col, self.row)

    def is_traversable(self) -> bool:
        """Return True if a drone can enter this cell."""
        return self.cell_type != CellType.OBSTACLE

    def __repr__(self) -> str:
        return f"Cell({self.col}, {self.row}, {self.cell_type.name})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Cell):
            return NotImplemented
        return self.col == other.col and self.row == other.row

    def __hash__(self) -> int:
        return hash((self.col, self.row))
