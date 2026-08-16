"""Lecture 16 -- Path planning (A* over an occupancy grid).

Run it:
    <isaac-sim>/python.sh lectures/lecture16.py

Lecture 15 turned one lidar scan into a grid of free/occupied/unknown
cells. That grid is only useful once something can act on it: given a
start cell and a goal cell, find a route through FREE cells that never
touches OCCUPIED or UNKNOWN -- both are "can't go there," for different
reasons: OCCUPIED because something is physically there, UNKNOWN because
the one scan that built this grid never confirmed it's clear. This
lecture adds one obstacle to Lecture 15's room, so the straight line
between start and goal is provably blocked, and runs A* to find what a
real detour costs.
"""

import heapq
import itertools

import os

import numpy as np
from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))

kit = SimulationApp({"headless": True, "enable_motion_bvh": True, "multi_gpu": False, "active_gpu": 0})

import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data  # noqa: E402
from isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo import CoordsType  # noqa: E402
from omni.replicator.core import Writer  # noqa: E402
from pxr import UsdGeom, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# Lecture 15's exact sealed room, reused byte-for-byte again -- plus one
# new prim, a pillar, sized floor-to-ceiling like the walls so the flat
# (elevation-0) 2D lidar is guaranteed to see it at every azimuth it
# blocks. Its footprint sits on the straight line between the start and
# goal cells chosen below, on purpose.
#
# Placement is constrained by something Lecture 11-15 never had to worry
# about: this lidar config's own nearRangeM is 1.0m (see the room's
# lidar/Example_Rotary_2D.json profile) -- anything closer than that
# returns nothing at all, not a close hit, and since the pillar would
# also physically occlude whatever's behind it, a pillar placed inside
# that 1m blind radius blacks out a whole wedge of the scan instead of
# showing up as an obstacle. (Caught by probing raw returns per-azimuth
# when a first attempt at (0.25,-0.5) -- the midpoint of a start/goal
# line that itself passes 0.5m from the sensor -- produced a ~90deg dead
# zone with zero returns, not the close ones a real obstacle gives.) The
# pillar and the start/goal line below all stay outside that radius.
cx, cy = 0.5, -0.5
half_x, half_y = 2.8, 3.8
wall_specs = [
    ("North", (cx, 3.0, 1.0), (2 * half_x, 0.2, 2.0)),
    ("South", (cx, -4.0, 1.0), (2 * half_x, 0.2, 2.0)),
    ("East", (3.0, cy, 1.0), (0.2, 2 * half_y, 2.0)),
    ("West", (-2.0, cy, 1.0), (0.2, 2 * half_y, 2.0)),
    ("Floor", (cx, cy, 0.0), (2 * half_x, 2 * half_y, 0.2)),
    ("Ceiling", (cx, cy, 2.0), (2 * half_x, 2 * half_y, 0.2)),
    ("Pillar", (0.25, 2.0, 1.0), (0.6, 0.6, 2.0)),
]
for name, pos, scale in wall_specs:
    wall = UsdGeom.Cube.Define(stage, f"/World/Wall{name}")
    wall.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(wall).SetTranslate(pos)
    UsdGeom.XformCommonAPI(wall).SetScale(scale)
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

lidar = Lidar.create(
    "/World/Lidar",
    config="Example_Rotary_2D",
    translations=np.array([0.0, 0.0, 1.0]),
    aux_output_level="BASIC",
)

captured = {"frames": [], "coords_type": None}


class CaptureWriter(Writer):
    """Same accumulation pattern as Lectures 11 and 15."""

    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]

    def write(self, data):
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            if gmo_raw is None or (hasattr(gmo_raw, "__len__") and len(gmo_raw) == 0):
                continue
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements == 0:
                continue
            captured["frames"].append({"x": np.array(gmo.x), "y": np.array(gmo.y), "z": np.array(gmo.z)})
            captured["coords_type"] = gmo.elementsCoordsType


rep.WriterRegistry.register(CaptureWriter)
sensor = LidarSensor(lidar, annotators=[])
sensor.attach_writer("CaptureWriter")

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(200):
    kit.update()
timeline.stop()

frames = captured["frames"]
if not frames or captured["coords_type"] != CoordsType.SPHERICAL:
    print("LECTURE: scan failed or elementsCoordsType is not SPHERICAL -- see lecture11.md")
    kit.close()
    raise SystemExit(1)

az_deg = np.concatenate([f["x"] for f in frames])
dist_m = np.concatenate([f["z"] for f in frames])
print(f"LECTURE: captured {len(az_deg)} beam returns, azimuth range "
      f"[{az_deg.min():.1f}, {az_deg.max():.1f}] deg, distance range "
      f"[{dist_m.min():.2f}, {dist_m.max():.2f}] m")

# =============================================================================
# Step 1-2: identical to Lecture 15 -- bin azimuth, keep nearest return per
# bin, rasterize the grid. The only thing new in the result is the pillar's
# footprint (OCCUPIED) and the wedge of cells behind it from the sensor's
# point of view, which the scan never confirmed (UNKNOWN).
# =============================================================================
N_BINS = 720
bin_edges = np.linspace(-180.0, 180.0, N_BINS + 1)
bin_idx = np.clip(np.digitize(az_deg, bin_edges) - 1, 0, N_BINS - 1)
bin_range = np.full(N_BINS, np.inf)
np.minimum.at(bin_range, bin_idx, dist_m)

RES = 0.2
X_MIN, X_MAX = -3.0, 4.0
Y_MIN, Y_MAX = -5.0, 4.0
xs = np.arange(X_MIN, X_MAX, RES)
ys = np.arange(Y_MIN, Y_MAX, RES)
grid_x, grid_y = np.meshgrid(xs + RES / 2, ys + RES / 2)

cell_r = np.sqrt(grid_x**2 + grid_y**2)
cell_theta = np.degrees(np.arctan2(grid_y, grid_x))
cell_bin = np.clip(np.digitize(cell_theta, bin_edges) - 1, 0, N_BINS - 1)
cell_range = bin_range[cell_bin]

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
grid = np.full(grid_x.shape, UNKNOWN, dtype=np.int8)
grid[cell_r < cell_range - RES / 2] = FREE
grid[np.abs(cell_r - cell_range) <= RES / 2] = OCCUPIED

n_free = int((grid == FREE).sum())
n_occ = int((grid == OCCUPIED).sum())
n_unk = int((grid == UNKNOWN).sum())
print(f"LECTURE: grid is {grid.shape[1]}x{grid.shape[0]} cells at {RES}m/cell -- "
      f"{n_free} free, {n_occ} occupied, {n_unk} unknown (Lecture 15 had no pillar: "
      f"759 free, 116 occupied, 700 unknown -- compare)")


def world_to_cell(wx: float, wy: float) -> tuple[int, int]:
    col = int(np.clip((wx - X_MIN) / RES, 0, grid.shape[1] - 1))
    row = int(np.clip((wy - Y_MIN) / RES, 0, grid.shape[0] - 1))
    return row, col


def cell_to_world(row: int, col: int) -> tuple[float, float]:
    return xs[col] + RES / 2, ys[row] + RES / 2


# =============================================================================
# Step 3: pick a start and goal on opposite sides of the room, and confirm
# in code -- not by eyeballing the map -- that the pillar actually sits on
# the straight line between them. If it didn't, this would be a lecture
# about drawing a line, not about why you need a planner at all.
# =============================================================================
start_world = (-1.5, 2.0)
goal_world = (2.0, 2.0)
start = world_to_cell(*start_world)
goal = world_to_cell(*goal_world)
print(f"LECTURE: start world={start_world} -> cell(row,col)={start}, grid value={int(grid[start])} (1=FREE)")
print(f"LECTURE: goal  world={goal_world} -> cell(row,col)={goal}, grid value={int(grid[goal])} (1=FREE)")
if grid[start] != FREE or grid[goal] != FREE:
    print("LECTURE: start or goal is not FREE -- pick different coordinates")
    kit.close()
    raise SystemExit(1)

n_samples = 200
straight_blocked = False
for t in np.linspace(0.0, 1.0, n_samples):
    wx = start_world[0] + t * (goal_world[0] - start_world[0])
    wy = start_world[1] + t * (goal_world[1] - start_world[1])
    if grid[world_to_cell(wx, wy)] != FREE:
        straight_blocked = True
        break
straight_line_m = float(np.hypot(goal_world[0] - start_world[0], goal_world[1] - start_world[1]))
print(f"LECTURE: straight line start->goal is {straight_line_m:.2f} m and "
      f"{'BLOCKED by the pillar' if straight_blocked else 'clear'} "
      f"(sampled {n_samples} points along it)")

# =============================================================================
# Step 4: A* over the grid, 8-connected. OCCUPIED and UNKNOWN are both
# impassable -- a real planner could treat unknown space as merely
# expensive instead of forbidden, but "don't drive where the sensor never
# confirmed it's clear" is the safer default and the one used here.
# =============================================================================
NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def astar(start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
    def h(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    counter = itertools.count()
    open_heap = [(h(start, goal), next(counter), start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return path[::-1]
        if current in closed:
            continue
        closed.add(current)
        for dr, dc in NEIGHBORS:
            nb = (current[0] + dr, current[1] + dc)
            if not (0 <= nb[0] < grid.shape[0] and 0 <= nb[1] < grid.shape[1]):
                continue
            if grid[nb] != FREE:
                continue
            step_cost = float(np.hypot(dr, dc))
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative_g
                heapq.heappush(open_heap, (tentative_g + h(nb, goal), next(counter), nb))
    return None


path = astar(start, goal)
if path is None:
    print("LECTURE: A* found no path -- start and goal are disconnected in this grid")
    kit.close()
    raise SystemExit(1)

# Verify: every cell FREE, every step 8-adjacent, endpoints correct.
assert all(grid[p] == FREE for p in path), "path touches a non-FREE cell"
assert all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1 for a, b in zip(path, path[1:])), "path has a non-adjacent step"
assert path[0] == start and path[-1] == goal, "path doesn't span start->goal"

path_len_m = sum(
    RES * np.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:])
)
print(f"LECTURE: A* path: {len(path)} cells, {path_len_m:.2f} m "
      f"(straight-line was {straight_line_m:.2f} m -- {path_len_m - straight_line_m:.2f} m of detour)")

# =============================================================================
# Step 5: print it. Same top-down orientation as Lecture 15, with the path
# drawn over the grid so you can see it actually goes around the pillar,
# not just report a number that claims it does.
# =============================================================================
CHARS = {UNKNOWN: " ", FREE: ".", OCCUPIED: "#"}
path_set = set(path)
print(f"\nLECTURE: occupancy grid with A* path ('#'=occupied '.'=free ' '=unknown "
      f"'*'=path 'S'=start 'G'=goal), x:[{X_MIN},{X_MAX}] y:[{Y_MIN},{Y_MAX}] at {RES}m/cell:")
for row in range(grid.shape[0] - 1, -1, -1):
    line = []
    for col in range(grid.shape[1]):
        cell = (row, col)
        if cell == start:
            line.append("S")
        elif cell == goal:
            line.append("G")
        elif cell in path_set:
            line.append("*")
        else:
            line.append(CHARS[grid[cell]])
    print("LECTURE: " + "".join(line))

# Raw grid + path data for tools/render_figures.py -- the actual A* result
# this run computed, not a redrawn illustration of it.
np.savez(
    os.path.join(HERE, "data_lecture16.npz"),
    grid=grid, x_min=X_MIN, x_max=X_MAX, y_min=Y_MIN, y_max=Y_MAX, res=RES,
    path=np.array(path), start=np.array(start), goal=np.array(goal),
)

kit.close()
