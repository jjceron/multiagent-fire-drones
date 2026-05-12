"""
Drone module — autonomous agent that navigates the grid to extinguish fires.
"""
from __future__ import annotations
from firedrones.agents.drone_state import DroneState
from firedrones import config


_drone_id_counter = 0


def _next_drone_id() -> int:
    global _drone_id_counter
    _drone_id_counter += 1
    return _drone_id_counter


class Drone:
    """
    An autonomous drone agent.

    Attributes:
        drone_id: Unique identifier.
        col, row: Current grid position.
        base_col, base_row: Home base position.
        state: Current DroneState.
        battery: Remaining battery (0–100).
        water: Remaining water (0–100).
        path: Planned sequence of (col, row) positions.
        target_fire_id: ID of the currently assigned fire, or None.
        extinguish_timer: Ticks spent at the fire cell (must reach 1 to complete).
        total_distance: Accumulated cells traveled.
        collisions_avoided: Number of times this drone waited to avoid collision.
    """

    def __init__(
        self,
        col: int,
        row: int,
        base_col: int | None = None,
        base_row: int | None = None,
    ) -> None:
        self.drone_id: int = _next_drone_id()
        self.col = col
        self.row = row
        self.base_col: int = base_col if base_col is not None else col
        self.base_row: int = base_row if base_row is not None else row
        self.state: DroneState = DroneState.IDLE
        self.battery: float = config.DRONE_BATTERY_MAX
        self.water: float = config.DRONE_WATER_MAX
        self.path: list[tuple[int, int]] = []
        self.target_fire_id: int | None = None
        self.extinguish_timer: int = 0
        self.total_distance: int = 0
        self.collisions_avoided: int = 0

    # ------------------------------------------------------------------
    # Position
    # ------------------------------------------------------------------

    @property
    def position(self) -> tuple[int, int]:
        return (self.col, self.row)

    @property
    def base_position(self) -> tuple[int, int]:
        return (self.base_col, self.base_row)

    def at_base(self) -> bool:
        return self.col == self.base_col and self.row == self.base_row

    # ------------------------------------------------------------------
    # Resource checks
    # ------------------------------------------------------------------

    def needs_recharge(self) -> bool:
        return self.battery <= config.LOW_BATTERY_THRESHOLD

    def needs_refill(self) -> bool:
        return self.water <= config.LOW_WATER_THRESHOLD

    def needs_resources(self) -> bool:
        return self.needs_recharge() or self.needs_refill()

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def move_to(self, col: int, row: int) -> None:
        """Move the drone one step and consume battery."""
        self.col = col
        self.row = row
        self.battery = max(0.0, self.battery - config.BATTERY_PER_MOVE)
        self.total_distance += 1

    def step_along_path(self) -> bool:
        """
        Advance one step along the planned path.
        Returns True if a step was taken, False if path is empty.
        """
        if not self.path:
            return False
        next_col, next_row = self.path.pop(0)
        self.move_to(next_col, next_row)
        return True

    # ------------------------------------------------------------------
    # Base operations
    # ------------------------------------------------------------------

    def recharge_tick(self) -> None:
        """Restore battery and water by one tick's worth at base."""
        self.battery = min(config.DRONE_BATTERY_MAX, self.battery + config.RECHARGE_RATE)
        self.water = min(config.DRONE_WATER_MAX, self.water + config.REFILL_RATE)

    def is_fully_recharged(self) -> bool:
        return (
            self.battery >= config.DRONE_BATTERY_MAX
            and self.water >= config.DRONE_WATER_MAX
        )

    # ------------------------------------------------------------------
    # Fire extinguishing
    # ------------------------------------------------------------------

    def consume_water(self) -> None:
        """Consume water to extinguish a fire."""
        self.water = max(0.0, self.water - config.WATER_PER_EXTINGUISH)

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def clear_assignment(self) -> None:
        """Reset task assignment and path."""
        self.target_fire_id = None
        self.path = []
        self.extinguish_timer = 0

    def __repr__(self) -> str:
        return (
            f"Drone(id={self.drone_id}, pos=({self.col},{self.row}), "
            f"state={self.state.name}, bat={self.battery:.0f}, water={self.water:.0f})"
        )
