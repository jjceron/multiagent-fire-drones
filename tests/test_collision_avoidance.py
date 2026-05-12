"""Tests for collision avoidance."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from firedrones.agents.drone import Drone
from firedrones.agents.drone_state import DroneState
from firedrones.planning.collision_avoidance import CollisionAvoidance


def make_moving_drone(drone_id_hint, col, row, next_col, next_row) -> Drone:
    """Create a drone with a one-step path."""
    d = Drone(col, row, 0, 0)
    d.state = DroneState.MOVING_TO_FIRE
    d.path = [(next_col, next_row)]
    return d


class TestCollisionAvoidance:
    def test_same_cell_collision_detected(self):
        """Two drones heading to the same cell: one must wait."""
        ca = CollisionAvoidance()
        d1 = make_moving_drone(1, 0, 5, 5, 5)
        d2 = make_moving_drone(2, 10, 5, 5, 5)

        must_wait = ca.resolve([d1, d2])
        assert len(must_wait) == 1, "Exactly one drone should wait"

    def test_no_collision_no_wait(self):
        """Drones heading to different cells should not be forced to wait."""
        ca = CollisionAvoidance()
        d1 = make_moving_drone(1, 0, 0, 1, 0)
        d2 = make_moving_drone(2, 5, 5, 6, 5)

        must_wait = ca.resolve([d1, d2])
        assert must_wait == set()

    def test_swap_collision_detected(self):
        """
        Drone A at (3,5) heading to (4,5), drone B at (4,5) heading to (3,5):
        this is a swap conflict — one drone must wait.
        """
        ca = CollisionAvoidance()
        d1 = make_moving_drone(1, 3, 5, 4, 5)
        d2 = make_moving_drone(2, 4, 5, 3, 5)

        must_wait = ca.resolve([d1, d2])
        assert len(must_wait) == 1, "Swap conflict: one drone should wait"

    def test_higher_id_waits(self):
        """The drone with the higher ID should be forced to wait."""
        ca = CollisionAvoidance()
        d1 = make_moving_drone(1, 0, 5, 5, 5)
        d2 = make_moving_drone(2, 10, 5, 5, 5)
        # Ensure d1.drone_id < d2.drone_id
        assert d1.drone_id < d2.drone_id

        must_wait = ca.resolve([d1, d2])
        assert d2.drone_id in must_wait

    def test_single_drone_no_collision(self):
        """A single drone can never collide with itself."""
        ca = CollisionAvoidance()
        d = make_moving_drone(1, 0, 0, 1, 0)
        must_wait = ca.resolve([d])
        assert must_wait == set()

    def test_collision_counter_incremented(self):
        """The waiting drone's collisions_avoided counter should increase."""
        ca = CollisionAvoidance()
        d1 = make_moving_drone(1, 0, 5, 5, 5)
        d2 = make_moving_drone(2, 10, 5, 5, 5)
        before = d2.collisions_avoided
        ca.resolve([d1, d2])
        assert d2.collisions_avoided == before + 1
