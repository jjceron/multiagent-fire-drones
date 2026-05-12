# FireDrones

**Academic project title:** Sistema Multi-Agente de Drones para Atención de Incendios Urbanos

---

## Description

FireDrones is a 2D grid-based multi-agent simulation where autonomous drones coordinate to respond to urban fires. The system models a rectangular workspace divided into square cells, featuring dynamic fire tasks, obstacle buildings, battery and water constraints, path planning with A\* and Dijkstra, greedy task allocation, basic collision avoidance, and a real-time pygame visualization.

---

## Assignment Context

This project is an academic exercise in **multi-agent robotic motion planning**. It demonstrates:

1. Multi-agent path planning (A\*, Dijkstra)
2. Task allocation (greedy nearest-agent assignment)
3. Basic combinatorial optimization (minimize total assignment cost)
4. Dynamic replanning (new fires appear mid-simulation)
5. Collision avoidance (wait-based strategy)
6. Battery and water resource management
7. GUI visualization with real-time metrics
8. Labeled test scenarios
9. Modular, testable code

---

## Features

- 30×20 grid workspace (configurable)
- 4 autonomous drone agents with state machines
- Fires dynamically spawn during the simulation
- A\* and Dijkstra pathfinding (switchable at runtime)
- Greedy task allocator (minimize total distance)
- Priority-based task assignment (toggle with `P`)
- Wait-based collision avoidance
- Battery and water resource constraints with auto return-to-base
- Pygame GUI with sidebar metrics, path visualization, and keyboard controls
- 8 labeled predefined scenarios
- 30+ unit and scenario tests

---

## Installation

> **Prerequisites:** Python 3.11+, existing `.venv` virtual environment.

```bash
# Activate the virtual environment
source .venv/bin/activate          # Linux / macOS
.venv\Scripts\activate             # Windows

# Install dependencies
pip install pygame pytest

# (Optional) install the package in editable mode
pip install -e .
```

---

## How to Run

```bash
# Full GUI simulation
python -m firedrones.main

# Headless mode (no pygame required, 50 ticks)
python -m firedrones.main --headless
```

---

## How to Run Tests

```bash
pytest
# or with verbose output
pytest -v
```

Tests do not require pygame. They run entirely headlessly.

---

## Keyboard Controls

| Key      | Action                              |
|----------|-------------------------------------|
| `Space`  | Pause / Resume                      |
| `R`      | Reset simulation                    |
| `F`      | Manually spawn a fire               |
| `O`      | Move a random obstacle              |
| `P`      | Toggle task priority mode           |
| `D`      | Toggle A\* / Dijkstra               |
| `LClick` | Spawn fire at clicked cell          |
| `Esc`    | Quit                                |

---

## Project Structure

```
FireDrones/
├── README.md
├── requirements.txt
├── pyproject.toml
├── src/
│   └── firedrones/
│       ├── __init__.py
│       ├── main.py            ← entry point
│       ├── config.py          ← all constants
│       ├── environment/
│       │   ├── cell.py        ← Cell, CellType
│       │   ├── grid.py        ← Grid
│       │   ├── region.py      ← Region
│       │   └── fire.py        ← Fire (task)
│       ├── agents/
│       │   ├── drone.py       ← Drone agent
│       │   └── drone_state.py ← DroneState enum
│       ├── planning/
│       │   ├── astar.py       ← A* planner
│       │   ├── dijkstra.py    ← Dijkstra planner
│       │   ├── task_allocator.py
│       │   └── collision_avoidance.py
│       ├── control/
│       │   └── controller.py  ← GUI ↔ Simulator mediator
│       ├── simulation/
│       │   ├── simulator.py   ← core simulation engine
│       │   ├── metrics.py     ← performance indicators
│       │   └── scenarios.py   ← 8 predefined scenarios
│       ├── gui/
│       │   └── pygame_gui.py  ← rendering and input
│       └── utils/
│           └── priority_queue.py
└── tests/
    ├── conftest.py
    ├── test_astar.py
    ├── test_dijkstra.py
    ├── test_grid.py
    ├── test_task_allocator.py
    ├── test_collision_avoidance.py
    └── test_scenarios.py
```

---

## Algorithms

### A\* Pathfinding

A\* is a best-first search algorithm that uses a heuristic `h(n)` to guide exploration toward the goal. For 4-connected grids with uniform move cost 1, Manhattan distance is an admissible and consistent heuristic:

```
f(n) = g(n) + h(n)
h(n) = |col_n - col_goal| + |row_n - row_goal|
```

A\* expands far fewer nodes than uninformed search on typical grid problems, making it the default planner. Time complexity: O(E log V).

### Dijkstra

Dijkstra is Uniform Cost Search with no heuristic (`h(n) = 0`). It expands all cells in non-decreasing distance order from the start, guaranteeing an optimal path on uniform-cost grids. It is included as a comparison baseline. On the same grid, Dijkstra and A\* produce paths of identical length, but Dijkstra visits more cells, making it slower.

**Why not RRT?** Rapidly-exploring Random Trees are designed for continuous configuration spaces. On a discrete grid, exhaustive search with a heuristic (A\*) is more appropriate and efficient. RRT would be relevant if the workspace were continuous or high-dimensional.

**Why not CBS (Conflict-Based Search)?** CBS solves the full Multi-Agent Path Finding (MAPF) problem optimally but has exponential worst-case complexity in the number of agents. It is outside the scope of this academic project; a simple wait-based collision avoidance strategy is sufficient here.

### Task Allocation

Greedy nearest-agent assignment:

1. Collect all active, unassigned fires.
2. Sort fires by priority (if priority mode is enabled).
3. For each fire, assign the nearest idle drone (measured by Manhattan distance).

### Priority Handling

When priority mode is active, fires are sorted by ascending `priority` value (0 = most urgent). The allocator attempts to assign the most urgent fire first. If a fire is unreachable (detected via a path query), it is skipped safely.

### Collision Avoidance

Before each movement tick:
1. Collect each drone's intended next cell.
2. If two drones intend to occupy the same cell, one must wait.
3. If drone A and B intend to swap positions, one must wait.
4. The drone with the higher ID waits (lower priority).

### Dynamic Replanning

When a new fire spawns or an obstacle moves, affected drone paths may be invalidated. At each tick, drones with an empty path (but still assigned a target) automatically replan using the current grid state.

### Battery and Water Constraints

| Parameter             | Value          |
|-----------------------|----------------|
| Battery per move      | 1              |
| Water per extinguish  | 25             |
| Low battery threshold | 25             |
| Low water threshold   | 25             |
| Recharge rate         | 10 / tick      |
| Refill rate           | 10 / tick      |

When battery or water drops below the threshold, the drone abandons its current task, returns to base, and recharges/refills before accepting new assignments.

---

## Optimization Formulation

### No-Priority Mode

Minimize total assignment cost:

```
J = Σ_{(i,j) ∈ A} distance(drone_i, fire_j)
```

where:
- `drone_i` is an available drone agent
- `fire_j` is an active, unassigned fire
- `distance` is Manhattan distance (proxy for travel time)
- `A` is the set of assigned agent–task pairs

The greedy algorithm approximates the optimal assignment by iterating fires and selecting the nearest available drone for each.

### Priority Mode

Tasks must be completed in strict ascending priority order whenever possible:

```
∀ fire_a, fire_b : priority(fire_a) < priority(fire_b)
  → extinguish_tick(fire_a) ≤ extinguish_tick(fire_b)   (when reachable)
```

If a higher-priority fire is surrounded by obstacles (unreachable), the system detects this via a path query and continues safely to the next-priority fire.

---

## Labeled Test Scenarios

| # | Name         | Description                                                  |
|---|--------------|--------------------------------------------------------------|
| 1 | scenario_1   | Single drone, single fire, no obstacles                      |
| 2 | scenario_2   | Single drone, wall of obstacles, drone must detour           |
| 3 | scenario_3   | 4 drones, 5 fires, greedy nearest-task allocation            |
| 4 | scenario_4   | 2 drones on collision course, avoidance triggered            |
| 5 | scenario_5   | Priority mode: high-priority fire served before low-priority |
| 6 | scenario_6   | Low battery → drone returns to base mid-mission             |
| 7 | scenario_7   | New fire spawns mid-simulation, drone replans                |
| 8 | scenario_8   | Fire surrounded by obstacles, system detects unreachability  |

---

## Limitations

- Collision avoidance is wait-based; no path-level MAPF solving.
- Task allocation is greedy (not globally optimal).
- No multi-fire triage (one drone per fire).
- No multi-base coordination (drones return to their own base).
- Grid movement is 4-connected (no diagonal moves).
- Pygame visualization requires a display (use `--headless` in CI).

---

## Future Improvements

- Replace greedy allocator with Hungarian algorithm for optimal assignment.
- Implement Conflict-Based Search for optimal multi-agent path planning.
- Add drone communication and shared fire maps.
- Support diagonal movement (8-connected grid).
- Add fire intensity levels and progressive extinguishing.
- Add real-time scenario loading via GUI.
- Export simulation logs for offline analysis.

---

## Real-World Applications

- **Emergency drones:** Coordinate UAV fleets for wildfire or building-fire suppression.
- **Warehouse robots:** Route multiple AGVs to pick locations while avoiding collisions.
- **Autonomous vehicles:** Multi-vehicle coordination in urban traffic grids.
- **Search and rescue:** Assign rescue drones to survivor locations with priority triage.
