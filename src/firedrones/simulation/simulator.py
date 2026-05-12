"""
Simulator — core simulation engine for FireDrones.

The Simulator owns the grid, drones, fires, and runs the main tick loop.
It is deliberately decoupled from pygame so it can be tested headlessly.
"""
from __future__ import annotations
import random
from firedrones.environment.grid import Grid
from firedrones.environment.cell import CellType
from firedrones.environment.fire import Fire, _fire_id_counter
from firedrones.agents.drone import Drone
from firedrones.agents.drone_state import DroneState
from firedrones.planning.astar import AStarPlanner
from firedrones.planning.dijkstra import DijkstraPlanner
from firedrones.planning.task_allocator import TaskAllocator
from firedrones.planning.collision_avoidance import CollisionAvoidance
from firedrones.simulation.metrics import Metrics
from firedrones.simulation.scenarios import ScenarioConfig
from firedrones import config


class Simulator:
    """
    Manages the full simulation state and runs each tick.

    Attributes:
        grid: The 2D workspace.
        drones: List of all drone agents.
        fires: List of all fire objects (active and extinguished).
        tick: Current simulation tick count.
        paused: Whether the simulation is paused.
        use_astar: True → A*, False → Dijkstra.
        priority_mode: Whether task priority is enforced.
        dynamic_obstacles: Whether obstacles can move.
        metrics: Current performance metrics snapshot.
    """

    def __init__(
        self,
        cols: int = config.GRID_COLS,
        rows: int = config.GRID_ROWS,
        num_drones: int = config.NUM_DRONES,
        num_obstacles: int = config.NUM_OBSTACLES,
        initial_fires: int = config.INITIAL_FIRES,
        fire_spawn_prob: float = config.FIRE_SPAWN_PROBABILITY,
        dynamic_obstacles: bool = config.DYNAMIC_OBSTACLES,
        priority_mode: bool = config.PRIORITY_MODE,
        use_astar: bool = config.USE_ASTAR,
        seed: int | None = None,
    ) -> None:
        self.rng = random.Random(seed)
        self.cols = cols
        self.rows = rows
        self.num_obstacles = num_obstacles
        self.fire_spawn_prob = fire_spawn_prob
        self.dynamic_obstacles = dynamic_obstacles
        self.priority_mode = priority_mode
        self.use_astar = use_astar
        self.tick: int = 0
        self.paused: bool = False
        self.metrics = Metrics()

        # Internals
        self._obstacle_move_timer: int = 0
        self._allocator = TaskAllocator()
        self._collision_avoidance = CollisionAvoidance()

        # Build initial state
        self.grid = Grid(cols, rows)
        self.drones: list[Drone] = []
        self.fires: list[Fire] = []
        self._base_positions: list[tuple[int, int]] = []

        self._initialize(num_drones, num_obstacles, initial_fires)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _initialize(
        self,
        num_drones: int,
        num_obstacles: int,
        initial_fires: int,
    ) -> None:
        """Set up grid, drones, obstacles, and initial fires."""
        self.grid.reset()

        # Place one base in each corner quadrant
        base_candidates = [
            (1, 1),
            (self.cols - 2, 1),
            (1, self.rows - 2),
            (self.cols - 2, self.rows - 2),
        ]
        num_bases = min(2, len(base_candidates))
        self._base_positions = base_candidates[:num_bases]
        for bc, br in self._base_positions:
            self.grid.set_type(bc, br, CellType.BASE)

        # Place random obstacles, avoiding base cells
        reserved = set(self._base_positions)
        empties = [
            c for c in self.grid.cells_of_type(CellType.EMPTY)
            if c.position not in reserved
        ]
        self.rng.shuffle(empties)
        for cell in empties[:num_obstacles]:
            cell.cell_type = CellType.OBSTACLE

        # Create drones at bases
        self.drones = []
        drone_index = 0
        from firedrones.agents import drone as drone_module
        drone_module._drone_id_counter = 0
        for i in range(num_drones):
            bc, br = self._base_positions[i % len(self._base_positions)]
            d = Drone(bc, br, bc, br)
            self.drones.append(d)
            drone_index += 1

        # Spawn initial fires on random empty cells
        self.fires = []
        self._spawn_fires(initial_fires)

    def _spawn_fires(self, count: int = 1) -> None:
        """Spawn `count` new fires on random empty cells."""
        occupied = {(d.col, d.row) for d in self.drones}
        occupied |= set(self._base_positions)
        occupied |= {f.position for f in self.fires if not f.extinguished}

        empties = [
            c for c in self.grid.cells_of_type(CellType.EMPTY)
            if c.position not in occupied
        ]
        self.rng.shuffle(empties)
        for cell in empties[:count]:
            fire = Fire(cell.col, cell.row, spawn_tick=self.tick)
            self.fires.append(fire)
            self.grid.set_type(cell.col, cell.row, CellType.FIRE)

    def spawn_fire_at(self, col: int, row: int) -> Fire | None:
        """Manually spawn a fire at a specific cell if empty."""
        cell = self.grid.get_cell(col, row)
        if cell and cell.cell_type == CellType.EMPTY:
            fire = Fire(col, row, spawn_tick=self.tick)
            self.fires.append(fire)
            self.grid.set_type(col, row, CellType.FIRE)
            return fire
        return None

    # ------------------------------------------------------------------
    # Planner
    # ------------------------------------------------------------------

    def _get_planner(self):
        if self.use_astar:
            return AStarPlanner(self.grid)
        return DijkstraPlanner(self.grid)

    def _is_reachable(self, start: tuple[int, int], goal: tuple[int, int]) -> bool:
        planner = self._get_planner()
        return len(planner.find_path(start, goal)) > 0 or start == goal

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------

    def step(self) -> None:
        """Advance the simulation by one tick."""
        if self.paused:
            return

        self.tick += 1

        # 1. Possibly spawn a new fire
        if self.rng.random() < self.fire_spawn_prob:
            self._spawn_fires(1)

        # 2. Optionally move obstacles
        if self.dynamic_obstacles:
            self._obstacle_move_timer += 1
            if self._obstacle_move_timer >= config.OBSTACLE_MOVE_INTERVAL:
                self._obstacle_move_timer = 0
                self._move_random_obstacle()

        # 3. Send drones needing resources back to base
        for drone in self.drones:
            if drone.state not in (DroneState.RETURNING_TO_BASE, DroneState.RECHARGING):
                if drone.needs_resources() and not drone.at_base():
                    self._send_to_base(drone)

        # 4. Assign unassigned fires to idle drones
        active_fires = [f for f in self.fires if not f.extinguished]
        assignments = self._allocator.allocate(
            self.drones,
            active_fires,
            priority_mode=self.priority_mode,
            reachable_fn=self._is_reachable,
        )
        for fire_id, drone_id in assignments.items():
            fire = next(f for f in self.fires if f.fire_id == fire_id)
            drone = next(d for d in self.drones if d.drone_id == drone_id)
            self._assign_drone_to_fire(drone, fire)

        # 5. Replan paths for drones whose path is outdated
        for drone in self.drones:
            if drone.state == DroneState.MOVING_TO_FIRE and not drone.path:
                fire = self._fire_for_drone(drone)
                if fire and not fire.extinguished:
                    self._plan_path(drone, fire.position)
                else:
                    drone.clear_assignment()
                    drone.state = DroneState.IDLE

        # 6. Collision avoidance — determine which drones must wait
        moving_drones = [
            d for d in self.drones
            if d.path and d.state in (
                DroneState.MOVING_TO_FIRE, DroneState.RETURNING_TO_BASE
            )
        ]
        must_wait = self._collision_avoidance.resolve(moving_drones)

        # 7. Move drones
        for drone in self.drones:
            if drone.drone_id in must_wait:
                continue  # wait this tick

            if drone.state == DroneState.MOVING_TO_FIRE and drone.path:
                drone.step_along_path()
                # Check arrival
                fire = self._fire_for_drone(drone)
                if fire and drone.position == fire.position:
                    drone.state = DroneState.EXTINGUISHING
                    drone.extinguish_timer = 0

            elif drone.state == DroneState.RETURNING_TO_BASE and drone.path:
                drone.step_along_path()
                if drone.at_base():
                    drone.state = DroneState.RECHARGING
                    drone.path = []

        # 8. Extinguishing — drone stays one tick at fire cell
        for drone in self.drones:
            if drone.state == DroneState.EXTINGUISHING:
                fire = self._fire_for_drone(drone)
                if fire and not fire.extinguished:
                    drone.extinguish_timer += 1
                    if drone.extinguish_timer >= config.EXTINGUISH_TICKS:
                        drone.consume_water()
                        fire.extinguish(self.tick)
                        self.grid.set_type(fire.col, fire.row, CellType.EMPTY)
                        drone.clear_assignment()
                        drone.state = DroneState.IDLE
                else:
                    # Fire already gone
                    drone.clear_assignment()
                    drone.state = DroneState.IDLE

        # 9. Recharge at base
        for drone in self.drones:
            if drone.state == DroneState.RECHARGING:
                drone.recharge_tick()
                if drone.is_fully_recharged():
                    drone.state = DroneState.IDLE

        # 10. Update metrics
        self.metrics.update(self.tick, self.drones, self.fires)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _assign_drone_to_fire(self, drone: Drone, fire: Fire) -> None:
        drone.target_fire_id = fire.fire_id
        fire.assigned_drone_id = drone.drone_id
        drone.state = DroneState.MOVING_TO_FIRE
        self._plan_path(drone, fire.position)

    def _send_to_base(self, drone: Drone) -> None:
        drone.clear_assignment()
        drone.state = DroneState.RETURNING_TO_BASE
        self._plan_path(drone, drone.base_position)

    def _plan_path(self, drone: Drone, goal: tuple[int, int]) -> None:
        planner = self._get_planner()
        path = planner.find_path(drone.position, goal)
        drone.path = path

    def _fire_for_drone(self, drone: Drone) -> Fire | None:
        if drone.target_fire_id is None:
            return None
        return next(
            (f for f in self.fires if f.fire_id == drone.target_fire_id),
            None,
        )

    def _move_random_obstacle(self) -> None:
        """Move one random obstacle to a random empty cell."""
        obstacles = self.grid.cells_of_type(CellType.OBSTACLE)
        empties = self.grid.cells_of_type(CellType.EMPTY)
        if not obstacles or not empties:
            return
        ob = self.rng.choice(obstacles)
        em = self.rng.choice(empties)
        ob.cell_type = CellType.EMPTY
        em.cell_type = CellType.OBSTACLE
        # Replan any affected drones
        for drone in self.drones:
            if drone.path:
                # Check if the new obstacle is on the path
                if em.position in drone.path:
                    target = drone.path[-1] if drone.path else None
                    if target:
                        drone.path = []  # Will be replanned next tick

    # ------------------------------------------------------------------
    # Scenario loading
    # ------------------------------------------------------------------

    def load_scenario(self, scenario: ScenarioConfig) -> None:
        """Reset and configure the simulator from a ScenarioConfig."""
        # Reset drone ID counter
        from firedrones.agents import drone as drone_module
        drone_module._drone_id_counter = 0
        from firedrones.environment import fire as fire_module
        fire_module._fire_id_counter = 0

        self.cols = scenario.cols
        self.rows = scenario.rows
        self.priority_mode = scenario.priority_mode
        self.dynamic_obstacles = scenario.dynamic_obstacles
        self.tick = 0
        self.fires = []
        self.drones = []

        self.grid = Grid(scenario.cols, scenario.rows)

        # Place obstacles
        for col, row in scenario.obstacle_positions:
            self.grid.set_type(col, row, CellType.OBSTACLE)

        # Place bases
        self._base_positions = scenario.base_positions or [(1, 1)]
        for bc, br in self._base_positions:
            self.grid.set_type(bc, br, CellType.BASE)

        # Create drones
        for col, row, bc, br in scenario.drone_positions:
            d = Drone(col, row, bc, br)
            self.drones.append(d)

        # Create fires
        for col, row, priority in scenario.fire_positions:
            fire = Fire(col, row, priority=priority, spawn_tick=0)
            self.fires.append(fire)
            self.grid.set_type(col, row, CellType.FIRE)

        self.metrics = Metrics()

    # ------------------------------------------------------------------
    # Controls
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reinitialize the simulation with the same parameters."""
        from firedrones.agents import drone as drone_module
        drone_module._drone_id_counter = 0
        from firedrones.environment import fire as fire_module
        fire_module._fire_id_counter = 0
        self._initialize(
            num_drones=len(self.drones) or config.NUM_DRONES,
            num_obstacles=self.num_obstacles,
            initial_fires=config.INITIAL_FIRES,
        )
        self.tick = 0
        self.metrics = Metrics()

    def toggle_pause(self) -> None:
        self.paused = not self.paused

    def toggle_priority_mode(self) -> None:
        self.priority_mode = not self.priority_mode

    def toggle_algorithm(self) -> None:
        self.use_astar = not self.use_astar

    def spawn_obstacle(self) -> None:
        """Randomly spawn or move an obstacle (for keyboard shortcut O)."""
        self._move_random_obstacle()

    def active_fires(self) -> list[Fire]:
        return [f for f in self.fires if not f.extinguished]
