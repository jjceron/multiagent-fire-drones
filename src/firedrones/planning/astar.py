"""
A* pathfinding algorithm for grid-based navigation.

A* is well-suited for discrete grid problems because it uses a heuristic
(Manhattan distance) to guide the search toward the goal, making it faster
than uninformed search algorithms such as Dijkstra in practice.

Time complexity: O(E log V) with a priority queue, where V is cells and
E is edges (at most 4 per cell in a 4-connected grid).
"""
from __future__ import annotations
from firedrones.environment.grid import Grid
from firedrones.utils.priority_queue import PriorityQueue


def _manhattan(c1: int, r1: int, c2: int, r2: int) -> int:
    """Manhattan distance heuristic (admissible for 4-connected grids)."""
    return abs(c1 - c2) + abs(r1 - r2)


class AStarPlanner:
    """
    Computes shortest paths on a Grid using the A* algorithm.

    The heuristic is Manhattan distance, which is admissible and consistent
    for 4-connected grids with uniform move cost 1.
    """

    def __init__(self, grid: Grid) -> None:
        self.grid = grid

    def find_path(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """
        Find the shortest path from start to goal.

        Args:
            start: (col, row) of the starting cell.
            goal:  (col, row) of the target cell.

        Returns:
            A list of (col, row) positions from start (exclusive) to goal
            (inclusive), or an empty list if no path exists.
        """
        sc, sr = start
        gc, gr = goal

        # Validate endpoints
        if not self.grid.in_bounds(sc, sr) or not self.grid.in_bounds(gc, gr):
            return []
        if not self.grid.is_traversable(gc, gr):
            return []
        if start == goal:
            return []

        # f = g + h
        open_set: PriorityQueue[tuple[int, int]] = PriorityQueue()
        open_set.push(start, 0)

        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        g_score: dict[tuple[int, int], float] = {start: 0.0}

        while not open_set.empty():
            current = open_set.pop()
            cc, cr = current

            if current == goal:
                return self._reconstruct(came_from, goal)

            for neighbour in self.grid.neighbours(cc, cr):
                nc, nr = neighbour.col, neighbour.row
                npos = (nc, nr)
                tentative_g = g_score[current] + 1.0

                if npos not in g_score or tentative_g < g_score[npos]:
                    g_score[npos] = tentative_g
                    f = tentative_g + _manhattan(nc, nr, gc, gr)
                    came_from[npos] = current
                    open_set.push(npos, f)

        return []  # no path found

    @staticmethod
    def _reconstruct(
        came_from: dict[tuple[int, int], tuple[int, int] | None],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """Reconstruct the path by following came_from pointers."""
        path: list[tuple[int, int]] = []
        node: tuple[int, int] | None = goal
        while node is not None and came_from.get(node) is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        return path
