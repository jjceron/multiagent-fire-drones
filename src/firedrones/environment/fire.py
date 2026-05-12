"""
Fire module — represents an active urban fire (task) in the simulation.
"""
from __future__ import annotations
import time


_fire_id_counter = 0


def _next_fire_id() -> int:
    global _fire_id_counter
    _fire_id_counter += 1
    return _fire_id_counter


class Fire:
    """
    Represents an active fire at a grid position.

    Attributes:
        fire_id: Unique identifier.
        col: Column position.
        row: Row position.
        priority: Integer priority (lower = higher urgency). Default 0.
        spawn_tick: Simulation tick when the fire appeared.
        extinguished: True once a drone has completed this task.
        assigned_drone_id: ID of the drone currently assigned to this fire.
    """

    def __init__(
        self,
        col: int,
        row: int,
        priority: int = 0,
        spawn_tick: int = 0,
    ) -> None:
        self.fire_id: int = _next_fire_id()
        self.col = col
        self.row = row
        self.priority = priority
        self.spawn_tick = spawn_tick
        self.extinguished: bool = False
        self.extinguish_tick: int | None = None
        self.assigned_drone_id: int | None = None

    @property
    def position(self) -> tuple[int, int]:
        return (self.col, self.row)

    def extinguish(self, tick: int) -> None:
        """Mark this fire as extinguished at the given tick."""
        self.extinguished = True
        self.extinguish_tick = tick

    def response_time(self) -> int | None:
        """Return the number of ticks from spawn to extinguishment."""
        if self.extinguish_tick is not None:
            return self.extinguish_tick - self.spawn_tick
        return None

    def __repr__(self) -> str:
        status = "extinguished" if self.extinguished else "active"
        return f"Fire(id={self.fire_id}, pos=({self.col},{self.row}), priority={self.priority}, {status})"
