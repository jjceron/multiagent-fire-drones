"""Shared pytest fixtures for FireDrones tests."""
import sys
import os

# Ensure src/ is on the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import pytest
from firedrones.environment.grid import Grid
from firedrones.environment.cell import CellType


@pytest.fixture
def small_grid() -> Grid:
    """A 10×10 grid with no obstacles."""
    return Grid(10, 10)


@pytest.fixture
def grid_with_wall() -> Grid:
    """A 10×10 grid with a vertical wall at col=5 (rows 0-8, leaving row 9 open)."""
    g = Grid(10, 10)
    for r in range(9):  # leave (5, 9) open so there IS a path
        g.set_type(5, r, CellType.OBSTACLE)
    return g


@pytest.fixture
def fully_blocked_grid() -> Grid:
    """A 10×10 grid where the goal cell is completely surrounded."""
    g = Grid(10, 10)
    # Surround (5, 5)
    for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        g.set_type(5 + dc, 5 + dr, CellType.OBSTACLE)
    return g
