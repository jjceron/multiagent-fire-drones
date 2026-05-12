"""
Basic collision avoidance for multi-drone navigation.

Two drones must not occupy the same cell at the same simulation step.
This module implements a simple wait-based strategy:
  - Detect which drones would collide on their next step.
  - Force the lower-priority drone (higher ID, or the one with more
    battery) to wait for one tick.
  - Detect and resolve direct position-swap conflicts.
"""
from __future__ import annotations
from firedrones.agents.drone import Drone


class CollisionAvoidance:
    """
    Detects and resolves potential collisions between drones.

    Strategy:
    - Before each movement step, collect the next intended position
      of every active drone.
    - If two drones plan to move to the same cell, one must wait.
    - If drone A plans to move to drone B's current cell AND drone B
      plans to move to drone A's current cell (swap), one must wait.
    - The drone that waits has its path preserved; it simply skips
      this tick's movement.
    """

    def resolve(self, drones: list[Drone]) -> set[int]:
        """
        Determine which drones must wait this tick.

        Args:
            drones: All drones with non-empty paths.

        Returns:
            A set of drone_ids that should NOT move this tick.
        """
        # Build map: drone_id → intended next position
        intentions: dict[int, tuple[int, int]] = {}
        for drone in drones:
            if drone.path:
                intentions[drone.drone_id] = drone.path[0]

        must_wait: set[int] = set()

        drone_ids = [d.drone_id for d in drones]

        for i, did_a in enumerate(drone_ids):
            for did_b in drone_ids[i + 1:]:
                if did_a in must_wait or did_b in must_wait:
                    continue

                pos_a = next((d.position for d in drones if d.drone_id == did_a), None)
                pos_b = next((d.position for d in drones if d.drone_id == did_b), None)
                next_a = intentions.get(did_a)
                next_b = intentions.get(did_b)

                if next_a is None or next_b is None:
                    continue

                collision = False

                # Same-cell collision
                if next_a == next_b:
                    collision = True

                # Swap collision: A→B's cell and B→A's cell simultaneously
                if next_a == pos_b and next_b == pos_a:
                    collision = True

                if collision:
                    # Lower-priority drone waits (higher ID = lower priority)
                    waiter = max(did_a, did_b)
                    must_wait.add(waiter)
                    # Increment collision avoided counter for the waiter
                    for drone in drones:
                        if drone.drone_id == waiter:
                            drone.collisions_avoided += 1
                            break

        return must_wait
