# Multi-Agent Drone System Simulator for Urban Firefighting

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

> **Prerequisites:** Python 3.11+ and an existing `.venv` virtual environment.

### Windows

```powershell
# Activate the virtual environment
.venv\Scripts\activate

# Install dependencies from requirements.txt
py -m pip install -r .\requirements.txt

# Install the project in editable mode
py -m pip install -e .
```

### Linux / macOS

```bash
# Activate the virtual environment
source .venv/bin/activate

# Install dependencies from requirements.txt
python -m pip install -r requirements.txt

# Install the project in editable mode
python -m pip install -e .
```

> Installing the project in editable mode allows Python to recognize the `firedrones` package inside the `src/` directory.

---

## How to Run

### Windows

```powershell
# Full GUI simulation
py -m firedrones.main

# Headless mode, without pygame window
py -m firedrones.main --headless
```

### Linux / macOS

```bash
# Full GUI simulation
python -m firedrones.main

# Headless mode, without pygame window
python -m firedrones.main --headless
```

---

## How to Run Tests

### Windows

```powershell
py -m pytest

# Or with verbose output
py -m pytest -v
```

### Linux / macOS

```bash
python -m pytest

# Or with verbose output
python -m pytest -v
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

```text
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
│       │   └── fire.py        ← Fire task
│       ├── agents/
│       │   ├── drone.py       ← Drone agent
│       │   └── drone_state.py ← DroneState enum
│       ├── planning/
│       │   ├── astar.py       ← A* planner
│       │   ├── dijkstra.py    ← Dijkstra planner
│       │   ├── task_allocator.py
│       │   └── collision_avoidance.py
│       ├── control/
│       │   └── controller.py  ← GUI and simulator mediator
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

A\* is a best-first search algorithm that uses a heuristic $h(n)$ to guide exploration toward the goal. For 4-connected grids with uniform movement cost 1, Manhattan distance is an admissible and consistent heuristic:


$$f(n) = g(n) + h(n)$$
$$h(n) = |col_n - col_{goal}| + |row_{n} - row_{goal}|$$


A\* expands fewer nodes than uninformed search on typical grid problems, making it the default planner.

### Dijkstra

Dijkstra is Uniform Cost Search with no heuristic, equivalent to using $h(n) = 0$. It expands cells in non-decreasing distance order from the start and guarantees an optimal path on uniform-cost grids.

On the same grid, Dijkstra and A\* produce paths of identical length, but Dijkstra usually visits more cells, making it slower.

**Why not RRT?** Rapidly-exploring Random Trees are designed for continuous configuration spaces. On a discrete grid, exhaustive search with a heuristic, such as A\*, is more appropriate and efficient.

**Why not CBS?** Conflict-Based Search solves the full Multi-Agent Path Finding problem optimally, but it has high computational complexity. It is outside the scope of this academic project.

---

## Task Allocation

The simulator uses greedy nearest-agent assignment:

1. Collect all active and unassigned fires.
2. Sort fires by priority if priority mode is enabled.
3. For each fire, assign the nearest available drone.
4. Estimate cost using Manhattan distance or planned path length.

---

## Priority Handling

When priority mode is active, fires are sorted by ascending `priority` value, where `0` represents the most urgent fire.

The allocator attempts to assign the most urgent fire first. If a high-priority fire is unreachable, the system skips it safely and continues operating.

---

## Collision Avoidance

Before each movement tick:

1. Each drone proposes its next cell.
2. If two drones try to occupy the same cell, one drone waits.
3. If two drones try to swap positions directly, one drone waits.
4. The drone with the higher ID waits.

This is a simple wait-based strategy, not a full optimal MAPF solver.

---

## Dynamic Replanning

When a new fire appears or the environment changes, drone paths may become outdated. At each simulation tick, drones with invalid or empty paths automatically replan using the current grid state.

---

## Battery and Water Constraints

| Parameter             | Value     |
|-----------------------|-----------|
| Battery per move      | 1         |
| Water per extinguish  | 25        |
| Low battery threshold | 25        |
| Low water threshold   | 25        |
| Recharge rate         | 10 / tick |
| Refill rate           | 10 / tick |

When battery or water drops below the threshold, the drone abandons its current task, returns to base, and recharges or refills before accepting new assignments.

---

## Optimization Formulation

### No-Priority Mode

The objective is to minimize total assignment cost:

$$J = \sum \text{distance}(drone_{i}, fire_{j})$$

where:

- `drone_i` is an available drone agent.
- `fire_j` is an active fire task.
- `distance` is Manhattan distance, A\* path length, or travel time.
- The assignment should reduce the total cost of serving all fires.

The greedy algorithm approximates the optimal assignment by assigning each fire to the nearest available drone.

### Priority Mode

Tasks should be completed in strict ascending priority order whenever possible:

```text
priority(fire_a) < priority(fire_b)
→ fire_a should be completed before fire_b
```

If a higher-priority fire is unreachable due to obstacles, the system detects this and continues safely.

---

## Labeled Test Scenarios

| # | Name         | Description                                                  |
|---|--------------|--------------------------------------------------------------|
| 1 | scenario_1   | Single drone, single fire, no obstacles                      |
| 2 | scenario_2   | Single drone, wall of obstacles, drone must detour           |
| 3 | scenario_3   | 4 drones, 5 fires, greedy nearest-task allocation            |
| 4 | scenario_4   | 2 drones on collision course, avoidance triggered            |
| 5 | scenario_5   | Priority mode: high-priority fire served before low-priority |
| 6 | scenario_6   | Low battery, drone returns to base mid-mission               |
| 7 | scenario_7   | New fire spawns mid-simulation, drone replans                |
| 8 | scenario_8   | Fire surrounded by obstacles, system detects unreachability  |

---

## Limitations

- Collision avoidance is wait-based.
- Task allocation is greedy and not globally optimal.
- No advanced MAPF solver is implemented.
- No multi-fire triage is included.
- One drone is assigned to one fire at a time.
- Grid movement is 4-connected, so diagonal movement is not supported.
- Pygame visualization requires a display. Use `--headless` when running without GUI support.

---

## Future Improvements

- Replace greedy allocation with the Hungarian algorithm.
- Implement Conflict-Based Search for optimal multi-agent path planning.
- Add drone-to-drone communication.
- Support diagonal movement with 8-connected grids.
- Add fire intensity and progressive extinguishing.
- Add real-time scenario loading through the GUI.
- Export simulation logs for offline analysis.
- Add multiple base coordination.

---

## Real-World Applications

- **Emergency drones:** Coordinate UAV fleets for wildfire or urban fire suppression.
- **Warehouse robots:** Route multiple robots to pick locations while avoiding collisions.
- **Autonomous vehicles:** Coordinate multiple vehicles in grid-like traffic environments.
- **Search and rescue:** Assign drones to emergency locations with priority triage.