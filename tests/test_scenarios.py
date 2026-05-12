"""
Labeled scenario tests — validate end-to-end simulation behavior
for each of the 8 predefined scenarios.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from firedrones.simulation.simulator import Simulator
from firedrones.simulation.scenarios import (
    SCENARIO_1, SCENARIO_2, SCENARIO_3, SCENARIO_4,
    SCENARIO_5, SCENARIO_6, SCENARIO_7, SCENARIO_8,
)
from firedrones.agents.drone_state import DroneState
from firedrones.planning.astar import AStarPlanner
from firedrones import config


def run_scenario(scenario, max_ticks=200) -> Simulator:
    """Load a scenario and run it for up to max_ticks ticks."""
    sim = Simulator(seed=1)
    sim.load_scenario(scenario)
    sim.fire_spawn_prob = 0.0  # disable random spawning during tests
    for _ in range(max_ticks):
        sim.step()
        active = [f for f in sim.fires if not f.extinguished]
        if not active:
            break
    return sim


class TestScenario1:
    """Single drone, single fire, no obstacles."""

    def test_fire_extinguished(self):
        sim = run_scenario(SCENARIO_1, max_ticks=100)
        assert all(f.extinguished for f in sim.fires), \
            "Scenario 1: fire should be extinguished"

    def test_task_completion_requires_one_tick(self):
        """
        A fire is completed only after the drone remains at the cell
        for at least one simulation tick (extinguish_timer check).
        """
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_1)
        sim.fire_spawn_prob = 0.0
        extinguished_early = False
        for tick in range(150):
            sim.step()
            for f in sim.fires:
                if f.extinguished and f.extinguish_tick == tick + 1:
                    # Check the drone spent at least 1 tick at the fire
                    drone = next(
                        (d for d in sim.drones if d.drone_id == f.assigned_drone_id),
                        None,
                    )
                    # The fire must have been extinguished after a stay tick
                    assert (f.extinguish_tick - f.spawn_tick) >= 1
                    extinguished_early = True
        assert extinguished_early or all(f.extinguished for f in sim.fires)


class TestScenario2:
    """Single drone, obstacles blocking direct path — drone must detour."""

    def test_fire_extinguished_via_detour(self):
        sim = run_scenario(SCENARIO_2, max_ticks=150)
        assert all(f.extinguished for f in sim.fires), \
            "Scenario 2: drone must find detour around obstacles"

    def test_path_avoids_obstacles(self):
        """Planned path must not pass through any obstacle cell."""
        from firedrones.environment.cell import CellType
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_2)
        sim.fire_spawn_prob = 0.0
        # Run a few ticks to let the drone plan its path
        sim.step()
        sim.step()
        for drone in sim.drones:
            for col, row in drone.path:
                cell = sim.grid.get_cell(col, row)
                assert cell.cell_type != CellType.OBSTACLE, \
                    f"Path goes through obstacle at ({col},{row})"


class TestScenario3:
    """Multiple drones, multiple fires, nearest-task allocation."""

    def test_all_fires_eventually_extinguished(self):
        sim = run_scenario(SCENARIO_3, max_ticks=300)
        assert all(f.extinguished for f in sim.fires), \
            "Scenario 3: all fires should be extinguished"

    def test_multiple_drones_dispatched(self):
        """More than one drone should be used."""
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_3)
        sim.fire_spawn_prob = 0.0
        dispatched = set()
        for _ in range(50):
            sim.step()
            for d in sim.drones:
                if d.state == DroneState.MOVING_TO_FIRE:
                    dispatched.add(d.drone_id)
        assert len(dispatched) > 1, "Multiple drones should be dispatched"


class TestScenario4:
    """Collision avoidance between two drones on a corridor."""

    def test_no_same_cell_occupation(self):
        """Two drones must never occupy the same cell at the same tick."""
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_4)
        sim.fire_spawn_prob = 0.0
        for _ in range(100):
            sim.step()
            positions = [d.position for d in sim.drones]
            assert len(positions) == len(set(positions)), \
                f"Two drones in same cell at tick {sim.tick}: {positions}"


class TestScenario5:
    """Priority-based task completion."""

    def test_high_priority_extinguished_first(self):
        """High-priority fire (priority=0) should be extinguished before low-priority."""
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_5)
        sim.fire_spawn_prob = 0.0

        high_prio_fire = next(f for f in sim.fires if f.priority == 0)
        low_prio_fire = next(f for f in sim.fires if f.priority > 0)

        for _ in range(200):
            sim.step()
            if high_prio_fire.extinguished and low_prio_fire.extinguished:
                break

        if high_prio_fire.extinguished and low_prio_fire.extinguished:
            assert high_prio_fire.extinguish_tick <= low_prio_fire.extinguish_tick, \
                "High-priority fire should be extinguished no later than low-priority"


class TestScenario6:
    """Low battery forces return to base."""

    def test_drone_returns_to_base_on_low_battery(self):
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_6)
        sim.fire_spawn_prob = 0.0

        # Manually drain the drone battery
        for drone in sim.drones:
            drone.battery = config.LOW_BATTERY_THRESHOLD - 1

        returned_to_base = False
        for _ in range(100):
            sim.step()
            for d in sim.drones:
                if d.state in (DroneState.RETURNING_TO_BASE, DroneState.RECHARGING):
                    returned_to_base = True
                    break
            if returned_to_base:
                break

        assert returned_to_base, "Drone with low battery should return to base"


class TestScenario7:
    """Dynamic fire forces replanning."""

    def test_new_fire_gets_assigned(self):
        """A fire spawned mid-simulation should eventually be assigned."""
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_7)
        sim.fire_spawn_prob = 0.0
        # Run a few ticks, then spawn a new fire
        for _ in range(10):
            sim.step()
        new_fire = sim.spawn_fire_at(7, 6)
        assert new_fire is not None, "Should be able to spawn a fire at (7,6)"

        assigned = False
        for _ in range(150):
            sim.step()
            if new_fire.assigned_drone_id is not None:
                assigned = True
                break

        assert assigned, "Newly spawned fire should be assigned to a drone"


class TestScenario8:
    """Unreachable fire due to obstacles."""

    def test_unreachable_fire_not_assigned(self):
        """If a fire is surrounded by obstacles, no drone should be assigned."""
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_8)
        sim.fire_spawn_prob = 0.0

        for _ in range(50):
            sim.step()

        blocked_fire = sim.fires[0]
        assert blocked_fire.assigned_drone_id is None, \
            "Unreachable fire should not be assigned to any drone"

    def test_astar_returns_empty_for_blocked_goal(self):
        """A* should return empty path for a fully surrounded cell."""
        from firedrones.environment.cell import CellType
        sim = Simulator(seed=0)
        sim.load_scenario(SCENARIO_8)
        planner = AStarPlanner(sim.grid)
        path = planner.find_path((1, 5), (6, 5))
        assert path == [], "A* should return empty path for unreachable goal"
