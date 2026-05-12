"""Tests for the TaskAllocator."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from firedrones.agents.drone import Drone
from firedrones.agents.drone_state import DroneState
from firedrones.environment.fire import Fire
from firedrones.planning.task_allocator import TaskAllocator


def make_drone(col, row, base_col=0, base_row=0) -> Drone:
    d = Drone(col, row, base_col, base_row)
    d.state = DroneState.IDLE
    d.target_fire_id = None
    return d


def make_fire(col, row, priority=0) -> Fire:
    return Fire(col, row, priority=priority, spawn_tick=0)


class TestTaskAllocatorNoPriority:
    def test_nearest_drone_assigned(self):
        """Drone nearest to the fire should be assigned."""
        allocator = TaskAllocator()
        drone_near = make_drone(1, 1)
        drone_far = make_drone(9, 9)
        fire = make_fire(2, 2)

        assignments = allocator.allocate([drone_near, drone_far], [fire])
        assert fire.fire_id in assignments
        assert assignments[fire.fire_id] == drone_near.drone_id

    def test_one_drone_per_fire(self):
        """Each fire gets at most one drone."""
        allocator = TaskAllocator()
        drones = [make_drone(0, 0), make_drone(5, 5)]
        fires = [make_fire(1, 1), make_fire(8, 8)]

        assignments = allocator.allocate(drones, fires)
        assigned_drones = list(assignments.values())
        # No drone should be assigned twice
        assert len(assigned_drones) == len(set(assigned_drones))

    def test_no_drones_available(self):
        """No assignments when no drones are idle."""
        allocator = TaskAllocator()
        d = make_drone(1, 1)
        d.state = DroneState.MOVING_TO_FIRE
        fire = make_fire(5, 5)

        assignments = allocator.allocate([d], [fire])
        assert assignments == {}

    def test_no_active_fires(self):
        """No assignments when all fires are extinguished."""
        allocator = TaskAllocator()
        drone = make_drone(1, 1)
        fire = make_fire(5, 5)
        fire.extinguished = True

        assignments = allocator.allocate([drone], [fire])
        assert assignments == {}

    def test_already_assigned_fire_skipped(self):
        """Fires already assigned to a drone are not re-assigned."""
        allocator = TaskAllocator()
        drone = make_drone(1, 1)
        fire = make_fire(5, 5)
        fire.assigned_drone_id = 99  # pretend already assigned

        assignments = allocator.allocate([drone], [fire])
        assert assignments == {}


class TestTaskAllocatorPriority:
    def test_priority_order_respected(self):
        """High-priority fire (lower value) should be assigned before low-priority."""
        allocator = TaskAllocator()
        # Single drone, two fires at equal distance
        drone = make_drone(5, 0)
        fire_low = make_fire(5, 5, priority=2)
        fire_high = make_fire(5, 3, priority=0)  # closer AND higher priority

        assignments = allocator.allocate(
            [drone], [fire_low, fire_high], priority_mode=True
        )
        # Only one drone, so only one fire is assigned
        assert len(assignments) == 1
        assigned_fire_id = list(assignments.keys())[0]
        assert assigned_fire_id == fire_high.fire_id, \
            "High-priority fire should be assigned first"

    def test_two_drones_priority_order(self):
        """With two drones, both fires get assigned; highest priority gets a drone first."""
        allocator = TaskAllocator()
        drone_a = make_drone(0, 0)
        drone_b = make_drone(9, 9)
        fire_high = make_fire(1, 1, priority=0)
        fire_low = make_fire(8, 8, priority=5)

        assignments = allocator.allocate(
            [drone_a, drone_b], [fire_high, fire_low], priority_mode=True
        )
        assert fire_high.fire_id in assignments
        assert fire_low.fire_id in assignments
        # Nearest drone to high-priority fire (drone_a) should handle it
        assert assignments[fire_high.fire_id] == drone_a.drone_id
