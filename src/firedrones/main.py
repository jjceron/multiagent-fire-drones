"""
FireDrones — entry point.

Run with:
    python -m firedrones.main

This starts the full pygame GUI simulation.
If pygame is not installed, a headless demo loop is run instead.
"""
from __future__ import annotations
import sys

from firedrones.simulation.simulator import Simulator
from firedrones.control.controller import Controller


def run_gui(controller: Controller) -> None:
    """Start the pygame GUI."""
    try:
        from firedrones.gui.pygame_gui import PygameGUI
        gui = PygameGUI(controller)
        gui.run()
    except RuntimeError as exc:
        print(f"[GUI] {exc}")
        print("[GUI] Falling back to headless mode (10 ticks).")
        run_headless(controller, ticks=10)


def run_headless(controller: Controller, ticks: int = 20) -> None:
    """Run the simulation without a GUI for a fixed number of ticks."""
    print("FireDrones — Headless simulation")
    print(f"Running {ticks} ticks …")
    for _ in range(ticks):
        controller.step()
    m = controller.metrics.as_dict()
    print("\n── Final metrics ──")
    for k, v in m.items():
        print(f"  {k:<25} {v}")


def main() -> None:
    simulator = Simulator(seed=42)
    controller = Controller(simulator)

    if "--headless" in sys.argv:
        run_headless(controller, ticks=50)
    else:
        run_gui(controller)


if __name__ == "__main__":
    main()
