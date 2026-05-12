"""Drone state enumeration."""
from enum import Enum, auto


class DroneState(Enum):
    """Possible operational states of an autonomous drone."""
    IDLE = auto()
    MOVING_TO_FIRE = auto()
    EXTINGUISHING = auto()
    RETURNING_TO_BASE = auto()
    RECHARGING = auto()
