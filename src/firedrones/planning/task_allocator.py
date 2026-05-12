"""
Task allocator — assigns fires (tasks) to drones (agents).

Objective (no-priority mode):
    Minimize total assignment cost:
        J = Σ distance(drone_i, fire_j)    for all assigned pairs (i, j)

    The greedy approach iterates over fires and assigns each one to the
    closest available drone, measured by Manhattan distance.

Priority mode:
    Tasks must be completed following a strict priority order.
    Lower priority values represent higher urgency (priority 0 is most urgent).
    If a higher-priority fire cannot be reached (e.g. surrounded by obstacles),
    the allocator skips it safely and continues with lower-priority fires.
"""
from __future__ import annotations
from firedrones.agents.drone import Drone
from firedrones.agents.drone_state import DroneState
from firedrones.environment.fire import Fire


def _manhattan(a: tuple[int, int], b: tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class TaskAllocator:
    """
    Greedy task allocator for multi-drone fire assignment.

    Strategy:
    - In no-priority mode: assign the nearest available drone to each fire,
      minimising total Manhattan distance (proxy for travel time).
    - In priority mode: sort fires by ascending priority value and apply
      the nearest-drone assignment within that order.

    If no drone is available for a given fire, it remains unassigned.
    """

    def allocate(
        self,
        drones: list[Drone],
        fires: list[Fire],
        priority_mode: bool = False,
        reachable_fn: "callable[[tuple,tuple], bool] | None" = None,
    ) -> dict[int, int]:
        """
        Assign drones to fires.

        Args:
            drones: All drones in the simulation.
            fires: Active (non-extinguished) fires.
            priority_mode: Whether to enforce fire priority order.
            reachable_fn: Optional callable(start, goal) → bool to skip
                          unreachable fires gracefully.

        Returns:
            A dict mapping fire_id → drone_id for each new assignment.
        """
        active_fires = [f for f in fires if not f.extinguished and f.assigned_drone_id is None]

        if priority_mode:
            active_fires = sorted(active_fires, key=lambda f: f.priority)

        # Available drones: IDLE or no current assignment
        available: list[Drone] = [
            d for d in drones
            if d.state == DroneState.IDLE and d.target_fire_id is None
        ]

        assignments: dict[int, int] = {}

        for fire in active_fires:
            if not available:
                break

            best_drone: Drone | None = None
            best_dist: int = int(1e9)

            for drone in available:
                if reachable_fn is not None:
                    if not reachable_fn(drone.position, fire.position):
                        continue  # skip unreachable
                dist = _manhattan(drone.position, fire.position)
                if dist < best_dist:
                    best_dist = dist
                    best_drone = drone

            if best_drone is not None:
                assignments[fire.fire_id] = best_drone.drone_id
                available.remove(best_drone)

        return assignments
