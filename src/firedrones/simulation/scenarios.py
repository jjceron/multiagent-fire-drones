"""
Labeled test scenarios for FireDrones.

Each scenario returns a SimulatorState-compatible dict that the Simulator
can load via Simulator.load_scenario(). Scenarios are also used in
test_scenarios.py for automated validation.
"""
from __future__ import annotations
from dataclasses import dataclass, field


@dataclass
class ScenarioConfig:
    """
    Describes the initial state of a simulation scenario.

    Attributes:
        name: Short identifier.
        description: Human-readable summary.
        cols: Grid columns.
        rows: Grid rows.
        drone_positions: List of (col, row, base_col, base_row).
        fire_positions: List of (col, row, priority).
        obstacle_positions: List of (col, row).
        base_positions: List of (col, row) for drone bases.
        dynamic_obstacles: Whether to enable dynamic obstacle movement.
        priority_mode: Whether task priority is enforced.
    """

    name: str
    description: str
    cols: int = 20
    rows: int = 15
    drone_positions: list[tuple[int, int, int, int]] = field(default_factory=list)
    fire_positions: list[tuple[int, int, int]] = field(default_factory=list)
    obstacle_positions: list[tuple[int, int]] = field(default_factory=list)
    base_positions: list[tuple[int, int]] = field(default_factory=list)
    dynamic_obstacles: bool = False
    priority_mode: bool = False


# ---------------------------------------------------------------------------
# Predefined scenarios
# ---------------------------------------------------------------------------

SCENARIO_1 = ScenarioConfig(
    name="scenario_1",
    description="Single drone, single fire, no obstacles. Basic path planning.",
    cols=10,
    rows=10,
    drone_positions=[(1, 1, 1, 1)],
    fire_positions=[(8, 8, 0)],
    obstacle_positions=[],
    base_positions=[(1, 1)],
)

SCENARIO_2 = ScenarioConfig(
    name="scenario_2",
    description="Single drone, obstacles blocking the direct path. Drone must detour.",
    cols=10,
    rows=10,
    drone_positions=[(1, 5, 1, 5)],
    fire_positions=[(8, 5, 0)],
    obstacle_positions=[(3, 4), (3, 5), (3, 6), (4, 4), (4, 6)],
    base_positions=[(1, 5)],
)

SCENARIO_3 = ScenarioConfig(
    name="scenario_3",
    description="Multiple drones, multiple fires, nearest-task greedy allocation.",
    cols=15,
    rows=12,
    drone_positions=[
        (1, 1, 1, 1),
        (13, 1, 13, 1),
        (1, 10, 1, 10),
        (13, 10, 13, 10),
    ],
    fire_positions=[
        (7, 6, 0),
        (3, 4, 0),
        (11, 4, 0),
        (3, 8, 0),
        (11, 8, 0),
    ],
    obstacle_positions=[(5, 5), (5, 6), (5, 7), (9, 5), (9, 6), (9, 7)],
    base_positions=[(1, 1)],
)

SCENARIO_4 = ScenarioConfig(
    name="scenario_4",
    description="Collision avoidance: two drones moving toward each other on the same corridor.",
    cols=12,
    rows=6,
    drone_positions=[
        (1, 3, 1, 3),
        (10, 3, 10, 3),
    ],
    fire_positions=[
        (10, 3, 0),  # drone 1 heads right
        (1, 3, 0),   # drone 2 heads left
    ],
    obstacle_positions=[
        (5, 0), (5, 1), (5, 2), (5, 4), (5, 5),  # narrow corridor at row 3
    ],
    base_positions=[(1, 3), (10, 3)],
)

SCENARIO_5 = ScenarioConfig(
    name="scenario_5",
    description="Priority-based task completion: high-priority fire is served first.",
    cols=12,
    rows=10,
    drone_positions=[(6, 5, 6, 5)],
    fire_positions=[
        (2, 5, 1),   # low priority
        (10, 5, 0),  # high priority (priority 0 = most urgent)
    ],
    obstacle_positions=[],
    base_positions=[(6, 5)],
    priority_mode=True,
)

SCENARIO_6 = ScenarioConfig(
    name="scenario_6",
    description="Low battery forces drone to return to base mid-mission.",
    cols=15,
    rows=10,
    drone_positions=[(1, 1, 1, 1)],
    fire_positions=[(13, 8, 0)],
    obstacle_positions=[],
    base_positions=[(1, 1)],
    # Battery will be set low programmatically in tests
)

SCENARIO_7 = ScenarioConfig(
    name="scenario_7",
    description="Dynamic fire: a new fire appears while the drone is already en route.",
    cols=15,
    rows=12,
    drone_positions=[(1, 1, 1, 1), (13, 1, 13, 1)],
    fire_positions=[(7, 10, 0)],
    obstacle_positions=[(4, 3), (4, 4), (4, 5), (10, 3), (10, 4), (10, 5)],
    base_positions=[(1, 1)],
)

SCENARIO_8 = ScenarioConfig(
    name="scenario_8",
    description="Unreachable fire: completely surrounded by obstacles. System detects and skips it.",
    cols=12,
    rows=10,
    drone_positions=[(1, 5, 1, 5)],
    fire_positions=[(6, 5, 0)],
    obstacle_positions=[
        (5, 4), (6, 4), (7, 4),
        (5, 5),         (7, 5),
        (5, 6), (6, 6), (7, 6),
    ],
    base_positions=[(1, 5)],
)

ALL_SCENARIOS: list[ScenarioConfig] = [
    SCENARIO_1,
    SCENARIO_2,
    SCENARIO_3,
    SCENARIO_4,
    SCENARIO_5,
    SCENARIO_6,
    SCENARIO_7,
    SCENARIO_8,
]

SCENARIO_MAP: dict[str, ScenarioConfig] = {s.name: s for s in ALL_SCENARIOS}
