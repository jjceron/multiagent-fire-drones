"""
Pygame GUI for FireDrones.

Handles all rendering and user input. The GUI is intentionally decoupled
from simulation logic — it reads state from the Controller and forwards
keyboard events back to it.

Keyboard controls:
  Space — pause / resume
  R     — reset simulation
  F     — manually spawn a fire
  O     — spawn / move an obstacle (dynamic obstacles)
  P     — toggle task priority mode
  D     — toggle A* / Dijkstra
  Esc   — quit
"""
from __future__ import annotations
import math
import sys

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

from firedrones import config
from firedrones.environment.cell import CellType
from firedrones.agents.drone_state import DroneState


class PygameGUI:
    """
    Pygame-based graphical front-end for the FireDrones simulator.

    The GUI owns the pygame window and clock. The Controller is passed
    in at construction time. Call run() to start the event loop.
    """

    def __init__(self, controller) -> None:
        if not PYGAME_AVAILABLE:
            raise RuntimeError(
                "pygame is not installed. Install it with: pip install pygame"
            )
        self.controller = controller
        self.cell_size = config.CELL_SIZE
        self.sidebar_width = config.SIDEBAR_WIDTH

        grid = controller.grid
        self.grid_pixel_w = grid.cols * self.cell_size
        self.grid_pixel_h = grid.rows * self.cell_size
        self.window_w = self.grid_pixel_w + self.sidebar_width
        self.window_h = self.grid_pixel_h

        pygame.init()
        pygame.display.set_caption(config.WINDOW_TITLE)
        self.screen = pygame.display.set_mode((self.window_w, self.window_h))
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_sm = pygame.font.SysFont("consolas", 13)
        self.font_md = pygame.font.SysFont("consolas", 15, bold=True)
        self.font_lg = pygame.font.SysFont("consolas", 18, bold=True)
        self.font_title = pygame.font.SysFont("consolas", 11)

        # Animation state
        self._tick_ms: int = config.SIMULATION_TICK_MS
        self._elapsed: int = 0

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start the main GUI loop."""
        running = True
        while running:
            dt = self.clock.tick(60)  # cap at 60 FPS

            # Process events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.controller.handle_event("pause")
                    elif event.key == pygame.K_r:
                        self.controller.handle_event("reset")
                    elif event.key == pygame.K_f:
                        self.controller.handle_event("spawn_fire")
                    elif event.key == pygame.K_o:
                        self.controller.handle_event("spawn_obstacle")
                    elif event.key == pygame.K_p:
                        self.controller.handle_event("toggle_priority")
                    elif event.key == pygame.K_d:
                        self.controller.handle_event("toggle_algorithm")

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # left click → spawn fire
                        mx, my = event.pos
                        if mx < self.grid_pixel_w:
                            col = mx // self.cell_size
                            row = my // self.cell_size
                            self.controller.handle_event("spawn_fire", col=col, row=row)

            # Advance simulation at configured tick rate
            self._elapsed += dt
            if self._elapsed >= self._tick_ms:
                self._elapsed = 0
                self.controller.step()

            # Render
            self._render()
            pygame.display.flip()

        pygame.quit()

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def _render(self) -> None:
        self.screen.fill(config.COLOR_BG)
        self._draw_grid()
        self._draw_paths()
        self._draw_drones()
        self._draw_sidebar()

    def _draw_grid(self) -> None:
        grid = self.controller.grid
        cs = self.cell_size

        for col in range(grid.cols):
            for row in range(grid.rows):
                cell = grid.get_cell(col, row)
                x = col * cs
                y = row * cs
                rect = pygame.Rect(x, y, cs, cs)

                if cell.cell_type == CellType.OBSTACLE:
                    color = config.COLOR_OBSTACLE
                    pygame.draw.rect(self.screen, color, rect)
                    # Subtle inner lines for "building" look
                    pygame.draw.rect(self.screen, (70, 70, 85), rect, 1)

                elif cell.cell_type == CellType.BASE:
                    pygame.draw.rect(self.screen, config.COLOR_BASE, rect)
                    # Landing pad cross
                    cx, cy = x + cs // 2, y + cs // 2
                    pygame.draw.line(self.screen, (160, 200, 255), (cx - 6, cy), (cx + 6, cy), 2)
                    pygame.draw.line(self.screen, (160, 200, 255), (cx, cy - 6), (cx, cy + 6), 2)

                elif cell.cell_type == CellType.FIRE:
                    self._draw_fire_cell(x, y, cs)

                else:
                    pygame.draw.rect(self.screen, config.COLOR_EMPTY, rect)
                    pygame.draw.rect(self.screen, config.COLOR_GRID_LINE, rect, 1)

        # Priority labels on fires
        if self.controller.priority_mode:
            for fire in self.controller.fires:
                if not fire.extinguished:
                    x = fire.col * cs + 2
                    y = fire.row * cs + 2
                    label = self.font_sm.render(f"P{fire.priority}", True, (255, 255, 200))
                    self.screen.blit(label, (x, y))

    def _draw_fire_cell(self, x: int, y: int, cs: int) -> None:
        """Draw an animated fire cell."""
        pygame.draw.rect(self.screen, config.COLOR_FIRE, pygame.Rect(x, y, cs, cs))
        # Glow dots
        cx, cy = x + cs // 2, y + cs // 2
        t = pygame.time.get_ticks() / 300.0
        for i in range(3):
            ox = int(4 * math.sin(t + i * 2.1))
            oy = int(4 * math.cos(t * 1.3 + i * 1.7))
            pygame.draw.circle(self.screen, config.COLOR_FIRE_GLOW, (cx + ox, cy + oy), 3)

    def _draw_paths(self) -> None:
        cs = self.cell_size
        half = cs // 2
        for drone in self.controller.drones:
            if not drone.path:
                continue
            color_index = (drone.drone_id - 1) % len(config.DRONE_COLORS)
            color = config.DRONE_COLORS[color_index]
            path_color = tuple(max(0, c - 80) for c in color)

            prev = (drone.col * cs + half, drone.row * cs + half)
            for col, row in drone.path:
                cur = (col * cs + half, row * cs + half)
                pygame.draw.line(self.screen, path_color, prev, cur, 1)
                prev = cur

    def _draw_drones(self) -> None:
        cs = self.cell_size
        half = cs // 2
        for drone in self.controller.drones:
            cx = drone.col * cs + half
            cy = drone.row * cs + half
            color_index = (drone.drone_id - 1) % len(config.DRONE_COLORS)
            color = config.DRONE_COLORS[color_index]

            # Dim color if low resources
            if drone.needs_resources():
                color = config.COLOR_DRONE_LOW

            radius = max(7, cs // 4)

            # Shadow
            pygame.draw.circle(self.screen, (0, 0, 0), (cx + 2, cy + 2), radius)
            # Body
            pygame.draw.circle(self.screen, color, (cx, cy), radius)
            # Outline
            pygame.draw.circle(self.screen, (255, 255, 255), (cx, cy), radius, 1)

            # State indicator dot
            state_color = self._state_color(drone.state)
            pygame.draw.circle(self.screen, state_color, (cx, cy), 3)

            # ID label
            label = self.font_sm.render(str(drone.drone_id), True, (240, 240, 240))
            self.screen.blit(label, (cx - 4, cy - 6))

    def _state_color(self, state: DroneState) -> tuple[int, int, int]:
        return {
            DroneState.IDLE: (180, 180, 180),
            DroneState.MOVING_TO_FIRE: (255, 200, 50),
            DroneState.EXTINGUISHING: (255, 100, 50),
            DroneState.RETURNING_TO_BASE: (100, 180, 255),
            DroneState.RECHARGING: (80, 255, 150),
        }.get(state, (200, 200, 200))

    def _draw_sidebar(self) -> None:
        sx = self.grid_pixel_w
        pygame.draw.rect(
            self.screen,
            config.COLOR_SIDEBAR_BG,
            pygame.Rect(sx, 0, self.sidebar_width, self.window_h),
        )

        # Divider line
        pygame.draw.line(
            self.screen, (50, 55, 75),
            (sx, 0), (sx, self.window_h), 2
        )

        y = 10
        pad = 12

        def draw_text(text: str, color=config.COLOR_TEXT, font=None, indent=0):
            nonlocal y
            f = font or self.font_sm
            surf = f.render(text, True, color)
            self.screen.blit(surf, (sx + pad + indent, y))
            y += surf.get_height() + 3

        def draw_divider():
            nonlocal y
            pygame.draw.line(
                self.screen, (45, 50, 68),
                (sx + 4, y), (sx + self.sidebar_width - 4, y), 1
            )
            y += 6

        # Title
        draw_text("FireDrones", config.COLOR_ACCENT, self.font_lg)
        draw_text("Multi-Agent Fire Response", config.COLOR_TEXT_DIM, self.font_title)
        y += 4
        draw_divider()

        # Algorithm & mode
        algo = "A*" if self.controller.use_astar else "Dijkstra"
        pmode = "ON" if self.controller.priority_mode else "off"
        paused = " [PAUSED]" if self.controller.paused else ""
        draw_text(f"Tick: {self.controller.tick}{paused}", config.COLOR_TEXT, self.font_md)
        draw_text(f"Algorithm : {algo}", config.COLOR_TEXT_DIM)
        draw_text(f"Priority  : {pmode}", config.COLOR_TEXT_DIM)
        y += 4
        draw_divider()

        # Metrics
        m = self.controller.metrics
        draw_text("── Metrics ──", config.COLOR_ACCENT, self.font_md)
        draw_text(f"Active fires     : {m.active_fires}")
        draw_text(f"Extinguished     : {m.extinguished_fires}")
        draw_text(f"Avg resp. time   : {m.avg_response_time:.1f} ticks")
        draw_text(f"Drones in mission: {m.drones_in_mission}")
        draw_text(f"Total battery    : {m.total_battery:.0f}")
        draw_text(f"Total water      : {m.total_water:.0f}")
        draw_text(f"Total distance   : {m.total_distance}")
        draw_text(f"Collisions avoid : {m.collisions_avoided}")
        y += 4
        draw_divider()

        # Drone status
        draw_text("── Drones ──", config.COLOR_ACCENT, self.font_md)
        for drone in self.controller.drones:
            color_index = (drone.drone_id - 1) % len(config.DRONE_COLORS)
            dc = config.DRONE_COLORS[color_index]
            state_short = {
                DroneState.IDLE: "IDLE",
                DroneState.MOVING_TO_FIRE: "→FIRE",
                DroneState.EXTINGUISHING: "EXTIN",
                DroneState.RETURNING_TO_BASE: "→BASE",
                DroneState.RECHARGING: "CHRG",
            }.get(drone.state, "?")
            text = (
                f"D{drone.drone_id} {state_short:5s} "
                f"bat:{drone.battery:3.0f} H2O:{drone.water:3.0f}"
            )
            draw_text(text, dc)
        y += 4
        draw_divider()

        # Controls
        draw_text("── Controls ──", config.COLOR_ACCENT, self.font_md)
        controls = [
            "Space  Pause / Resume",
            "R      Reset",
            "F      Spawn fire",
            "O      Move obstacle",
            "P      Toggle priority",
            "D      Toggle algorithm",
            "LClick Spawn fire here",
            "Esc    Quit",
        ]
        for line in controls:
            draw_text(line, config.COLOR_TEXT_DIM)
