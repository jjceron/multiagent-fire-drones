"""
Metrics module — tracks simulation performance indicators.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from firedrones.agents.drone import Drone
from firedrones.environment.fire import Fire


@dataclass
class Metrics:
    """Snapshot of simulation statistics at a given tick."""

    tick: int = 0
    active_fires: int = 0
    extinguished_fires: int = 0
    avg_response_time: float = 0.0
    drones_in_mission: int = 0
    total_water: float = 0.0
    total_battery: float = 0.0
    total_distance: int = 0
    collisions_avoided: int = 0

    def update(self, tick: int, drones: list[Drone], fires: list[Fire]) -> None:
        """Recompute all metrics from current simulation state."""
        from firedrones.agents.drone_state import DroneState

        self.tick = tick
        self.active_fires = sum(1 for f in fires if not f.extinguished)
        self.extinguished_fires = sum(1 for f in fires if f.extinguished)

        response_times = [
            f.response_time()
            for f in fires
            if f.extinguished and f.response_time() is not None
        ]
        self.avg_response_time = (
            sum(response_times) / len(response_times) if response_times else 0.0
        )

        self.drones_in_mission = sum(
            1 for d in drones if d.state not in (DroneState.IDLE, DroneState.RECHARGING)
        )
        self.total_water = sum(d.water for d in drones)
        self.total_battery = sum(d.battery for d in drones)
        self.total_distance = sum(d.total_distance for d in drones)
        self.collisions_avoided = sum(d.collisions_avoided for d in drones)

    def as_dict(self) -> dict[str, object]:
        return {
            "tick": self.tick,
            "active_fires": self.active_fires,
            "extinguished_fires": self.extinguished_fires,
            "avg_response_time": round(self.avg_response_time, 1),
            "drones_in_mission": self.drones_in_mission,
            "total_water": round(self.total_water, 1),
            "total_battery": round(self.total_battery, 1),
            "total_distance": self.total_distance,
            "collisions_avoided": self.collisions_avoided,
        }
