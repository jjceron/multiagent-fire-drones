"""
Controller — connects the Simulator to the GUI.

The Controller owns the Simulator and acts as a mediator:
  - It exposes the simulation state to the GUI for rendering.
  - It forwards user input events to the Simulator.
  - It drives the simulation loop (tick timing).
"""
from __future__ import annotations
from firedrones.simulation.simulator import Simulator
from firedrones.simulation.scenarios import ScenarioConfig
from firedrones import config


class Controller:
    """
    Mediator between the Simulator and the GUI.

    The GUI calls handle_event() with parsed keyboard events.
    The GUI reads state through simulator properties.
    """

    def __init__(self, simulator: Simulator | None = None) -> None:
        self.simulator: Simulator = simulator or Simulator()

    # ------------------------------------------------------------------
    # Event handling (called by GUI)
    # ------------------------------------------------------------------

    def handle_event(self, event: str, **kwargs) -> None:
        """
        Dispatch a named event to the simulator.

        Supported events:
            'pause'     — toggle pause/resume
            'reset'     — reset simulation
            'spawn_fire'— manually spawn a fire at optional (col, row)
            'spawn_obstacle' — spawn/move an obstacle
            'toggle_priority' — toggle priority mode
            'toggle_algorithm' — toggle A* / Dijkstra
            'quit'      — signal application exit
            'load_scenario' — load a ScenarioConfig (pass scenario=...)
        """
        sim = self.simulator

        if event == "pause":
            sim.toggle_pause()

        elif event == "reset":
            sim.reset()

        elif event == "spawn_fire":
            col = kwargs.get("col")
            row = kwargs.get("row")
            if col is not None and row is not None:
                sim.spawn_fire_at(col, row)
            else:
                sim._spawn_fires(1)

        elif event == "spawn_obstacle":
            sim.spawn_obstacle()

        elif event == "toggle_priority":
            sim.toggle_priority_mode()

        elif event == "toggle_algorithm":
            sim.toggle_algorithm()

        elif event == "load_scenario":
            scenario: ScenarioConfig = kwargs["scenario"]
            sim.load_scenario(scenario)

        # 'quit' is handled by the GUI loop itself

    # ------------------------------------------------------------------
    # State accessors (for GUI rendering)
    # ------------------------------------------------------------------

    @property
    def grid(self):
        return self.simulator.grid

    @property
    def drones(self):
        return self.simulator.drones

    @property
    def fires(self):
        return self.simulator.fires

    @property
    def metrics(self):
        return self.simulator.metrics

    @property
    def tick(self) -> int:
        return self.simulator.tick

    @property
    def paused(self) -> bool:
        return self.simulator.paused

    @property
    def priority_mode(self) -> bool:
        return self.simulator.priority_mode

    @property
    def use_astar(self) -> bool:
        return self.simulator.use_astar

    def step(self) -> None:
        """Advance the simulation by one tick."""
        self.simulator.step()
