"""
FireDrones - Global configuration and default simulation settings.
"""

# --- Grid ---
GRID_COLS: int = 30
GRID_ROWS: int = 20
CELL_SIZE: int = 32  # pixels per cell

# --- Drones ---
NUM_DRONES: int = 4
DRONE_BATTERY_MAX: float = 100.0
DRONE_WATER_MAX: float = 100.0
BATTERY_PER_MOVE: float = 1.0       # battery consumed per cell moved
WATER_PER_EXTINGUISH: float = 25.0  # water consumed per fire extinguished
LOW_BATTERY_THRESHOLD: float = 25.0
LOW_WATER_THRESHOLD: float = 25.0
RECHARGE_RATE: float = 10.0         # battery restored per tick at base
REFILL_RATE: float = 10.0           # water restored per tick at base

# --- Fires ---
INITIAL_FIRES: int = 3
FIRE_SPAWN_PROBABILITY: float = 0.02  # probability each tick a new fire spawns
EXTINGUISH_TICKS: int = 1             # ticks the drone must stay at fire cell

# --- Obstacles ---
NUM_OBSTACLES: int = 20
DYNAMIC_OBSTACLES: bool = False
OBSTACLE_MOVE_INTERVAL: int = 50     # ticks between random obstacle moves

# --- Simulation ---
SIMULATION_TICK_MS: int = 150        # milliseconds per tick
PRIORITY_MODE: bool = False
USE_ASTAR: bool = True               # False → use Dijkstra

# --- GUI ---
SIDEBAR_WIDTH: int = 260
WINDOW_TITLE: str = "FireDrones — Sistema Multi-Agente de Drones"

# --- Colors (RGB) ---
COLOR_BG = (18, 18, 28)
COLOR_EMPTY = (30, 32, 48)
COLOR_OBSTACLE = (55, 55, 70)
COLOR_BASE = (30, 100, 200)
COLOR_FIRE = (220, 60, 20)
COLOR_FIRE_GLOW = (255, 130, 0)
COLOR_DRONE = (80, 220, 120)
COLOR_DRONE_LOW = (220, 200, 50)
COLOR_PATH = (80, 160, 255)
COLOR_SIDEBAR_BG = (12, 12, 20)
COLOR_TEXT = (210, 215, 230)
COLOR_TEXT_DIM = (110, 115, 130)
COLOR_ACCENT = (80, 200, 255)
COLOR_GRID_LINE = (38, 40, 58)
COLOR_EXTINGUISHED = (60, 160, 80)

# Drone color palette (one per drone index)
DRONE_COLORS = [
    (80, 220, 120),
    (255, 190, 60),
    (120, 180, 255),
    (255, 100, 160),
    (180, 120, 255),
    (80, 230, 210),
    (255, 160, 80),
    (200, 255, 100),
]
