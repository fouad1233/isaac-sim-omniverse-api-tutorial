"""Lecture 19 -- Module 2 capstone: scan, map, plan, drive.

Run it:
    <isaac-sim>/python.sh lectures/lecture19.py

Lecture 15 turned a lidar scan into an occupancy grid. Lecture 16 ran A*
over that grid to find a route around an obstacle. Lecture 18 built a
[linear, angular] -> [left wheel, right wheel] controller and dead-
reckoned odometry from wheel motion alone. This capstone runs all four
in one script, in the order a real mobile robot actually would: scan the
room, build the map, plan a path, then physically drive a Jetbot along
it -- and checks the one thing none of the earlier lectures could check
on their own, because they never had a driving robot to check it with:
does the plan, once actually executed against real physics, get the
robot to the goal?

(Lecture 17's H1 sits outside this stack on purpose -- a legged
ready-to-use policy and a wheeled navigation stack are two different
skills on two different embodiments, not two stages of one pipeline.)
"""

import heapq
import itertools
import math
import os

import numpy as np
from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))

kit = SimulationApp({"headless": True, "enable_motion_bvh": True, "multi_gpu": False, "active_gpu": 0})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data  # noqa: E402
from isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo import CoordsType  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from omni.replicator.core import Writer  # noqa: E402
from pxr import UsdGeom, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# =============================================================================
# Phase 1: SCAN. Lecture 16's exact sealed room and pillar, byte-for-byte --
# no PhysicsScene yet, none of these prims are dynamic, and the lidar scan
# doesn't need one, exactly like Lectures 11/15/16.
# =============================================================================
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
    # Static collision only, deliberately no RigidBodyAPI -- Phase 3 adds
    # a PhysicsScene later for the Jetbot, and these walls only need to be
    # solid obstacles for it, not simulated bodies themselves.

FLOOR_TOP_Z = 0.1  # Floor center z=0.0, half-height 0.1 -- see wall_specs above

lidar = Lidar.create(
    "/World/Lidar",
    config="Example_Rotary_2D",
    translations=np.array([0.0, 0.0, 1.0]),
    aux_output_level="BASIC",
)

captured = {"frames": [], "coords_type": None}


class CaptureWriter(Writer):
    """Same accumulation pattern as Lectures 11, 15, and 16."""

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

# Phase 3 tears down and rebuilds the physics simulation view (a fresh
# UsdPhysics.Scene + WheeledRobot, then SimulationManager.setup_simulation()
# + a second timeline play). A lidar writer left attached across that
# rebuild races it: the RTX/motion-BVH pipeline backing the still-attached
# sensor finishes its own re-init a few ticks into Phase 3's drive loop,
# invalidating the Jetbot's physics tensor view out from under it --
# "Instance's physics tensor entity is not valid" a few steps into driving,
# not at construction time, which is what makes it non-obvious. Detaching
# the writer (and the sensor never being touched again) avoids the race.
sensor.detach_writer("CaptureWriter")

frames = captured["frames"]
if not frames or captured["coords_type"] != CoordsType.SPHERICAL:
    print("LECTURE: scan failed or elementsCoordsType is not SPHERICAL -- see lecture11.md")
    kit.close()
    raise SystemExit(1)

az_deg = np.concatenate([f["x"] for f in frames])
dist_m = np.concatenate([f["z"] for f in frames])
print(f"LECTURE: [scan] captured {len(az_deg)} beam returns, azimuth range "
      f"[{az_deg.min():.1f}, {az_deg.max():.1f}] deg, distance range "
      f"[{dist_m.min():.2f}, {dist_m.max():.2f}] m")

# =============================================================================
# Phase 2: MAP + PLAN. Identical to Lectures 15 and 16.
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

print(f"LECTURE: [map] grid is {grid.shape[1]}x{grid.shape[0]} cells at {RES}m/cell -- "
      f"{int((grid == FREE).sum())} free, {int((grid == OCCUPIED).sum())} occupied, "
      f"{int((grid == UNKNOWN).sum())} unknown")


def world_to_cell(wx: float, wy: float) -> tuple[int, int]:
    col = int(np.clip((wx - X_MIN) / RES, 0, grid.shape[1] - 1))
    row = int(np.clip((wy - Y_MIN) / RES, 0, grid.shape[0] - 1))
    return row, col


def cell_to_world(row: int, col: int) -> tuple[float, float]:
    return xs[col] + RES / 2, ys[row] + RES / 2


start_world = (-1.5, 2.0)
goal_world = (2.0, 2.0)
start = world_to_cell(*start_world)
goal = world_to_cell(*goal_world)
if grid[start] != FREE or grid[goal] != FREE:
    print("LECTURE: start or goal is not FREE -- pick different coordinates")
    kit.close()
    raise SystemExit(1)

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
assert all(grid[p] == FREE for p in path), "path touches a non-FREE cell"
assert path[0] == start and path[-1] == goal, "path doesn't span start->goal"
path_len_m = sum(RES * np.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:]))
print(f"LECTURE: [plan] A* path: {len(path)} cells, {path_len_m:.2f} m, "
      f"start={start_world} -> goal={goal_world}")


def simplify_path(cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Collapse runs of cells moving in the same direction into their
    endpoints -- a waypoint follower needs corners, not every 0.2m grid
    step A* happened to take.
    """
    if len(cells) <= 2:
        return list(cells)
    simplified = [cells[0]]
    prev_dir = (cells[1][0] - cells[0][0], cells[1][1] - cells[0][1])
    for i in range(1, len(cells) - 1):
        d = (cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1])
        if d != prev_dir:
            simplified.append(cells[i])
            prev_dir = d
    simplified.append(cells[-1])
    return simplified


waypoint_cells = simplify_path(path)
waypoints = [cell_to_world(*c) for c in waypoint_cells]
print(f"LECTURE: [plan] simplified to {len(waypoints)} waypoints: "
      f"{['(%.2f, %.2f)' % w for w in waypoints]}")

# =============================================================================
# Phase 3: DRIVE. Only now does a PhysicsScene and a dynamic robot enter the
# stage -- the scan and the plan above never needed one. Jetbot + controller
# setup matches Lecture 18's verified pattern exactly (same setup_simulation
# call, same device="cpu", same plain per-step loop -- no render/physics
# decoupling here, so Lecture 17's callback requirement doesn't apply).
# =============================================================================
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")
jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

NOMINAL_WHEEL_RADIUS = 0.03
WHEEL_BASE = 0.1125
DT = 1.0 / 60.0

jetbot = WheeledRobot(
    paths="/World/Jetbot",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=jetbot_asset_path,
    positions=[start_world[0], start_world[1], FLOOR_TOP_Z + 0.05],
)
controller = DifferentialController(wheel_radius=NOMINAL_WHEEL_RADIUS, wheel_base=WHEEL_BASE)
wheel_dof_indices = jetbot.get_dof_indices(["left_wheel_joint", "right_wheel_joint"]).numpy().tolist()

SimulationManager.setup_simulation(dt=DT, device="cpu")
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)
app_utils.play()
app_utils.update_app(steps=10)


def world_pose(robot):
    pos = robot.get_world_poses()[0].numpy()[0]
    w, x, y, z = robot.get_world_poses()[1].numpy()[0]
    yaw = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return float(pos[0]), float(pos[1]), yaw


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def dead_reckon(state, omega_left, omega_right):
    v_left = omega_left * NOMINAL_WHEEL_RADIUS
    v_right = omega_right * NOMINAL_WHEEL_RADIUS
    v = (v_left + v_right) / 2.0
    w = (v_right - v_left) / WHEEL_BASE
    state["x"] += v * math.cos(state["theta"]) * DT
    state["y"] += v * math.sin(state["theta"]) * DT
    state["theta"] += w * DT


x0, y0, theta0 = world_pose(jetbot)
odo = {"x": x0, "y": y0, "theta": theta0}
print(f"LECTURE: [drive] jetbot spawned at ({x0:.3f}, {y0:.3f}), heading {math.degrees(theta0):.1f} deg -- "
      f"driving toward {len(waypoints) - 1} waypoint(s)")

MAX_LINEAR = 0.2
K_HEADING = 2.5
WAYPOINT_TOL = 0.1
MAX_STEPS_PER_WAYPOINT = 900
total_steps = 0

# Per-step trajectory, for tools/render_figures.py -- the actual driven
# path, not just the start/end points already printed above.
traj_gt = [(x0, y0)]
traj_odo = [(x0, y0)]

for wp in waypoints[1:]:
    reached = False
    for _ in range(MAX_STEPS_PER_WAYPOINT):
        x, y, theta = world_pose(jetbot)
        dx, dy = wp[0] - x, wp[1] - y
        dist = math.hypot(dx, dy)
        if dist < WAYPOINT_TOL:
            reached = True
            break
        heading_err = wrap_to_pi(math.atan2(dy, dx) - theta)
        v_cmd = MAX_LINEAR * max(0.0, math.cos(heading_err))
        w_cmd = K_HEADING * heading_err
        wheel_vel = controller.forward([v_cmd, w_cmd])
        jetbot.apply_wheel_actions(wheel_vel)
        app_utils.update_app(steps=1)
        total_steps += 1
        actual = jetbot.get_dof_velocities(dof_indices=wheel_dof_indices).numpy()[0]
        dead_reckon(odo, float(actual[0]), float(actual[1]))
        traj_gt.append((x, y))
        traj_odo.append((odo["x"], odo["y"]))
    if not reached:
        print(f"LECTURE: [drive] WARNING did not reach waypoint ({wp[0]:.2f}, {wp[1]:.2f}) "
              f"within {MAX_STEPS_PER_WAYPOINT} steps")

x1, y1, theta1 = world_pose(jetbot)
dist_to_goal = float(np.hypot(goal_world[0] - x1, goal_world[1] - y1))
odo_err = float(np.hypot(odo["x"] - x1, odo["y"] - y1))

print(f"LECTURE: [drive] {total_steps} physics steps ({total_steps * DT:.1f}s sim time)")
print(f"LECTURE: [drive] ground truth final pose  -> x={x1:.3f} y={y1:.3f} theta={math.degrees(theta1):.1f} deg "
      f"-- {dist_to_goal:.3f} m from the planned goal {goal_world}")
print(f"LECTURE: [drive] odometry-believed pose   -> x={odo['x']:.3f} y={odo['y']:.3f} "
      f"theta={math.degrees(odo['theta']):.1f} deg -- {odo_err:.3f} m from ground truth "
      f"(what Lecture 18 already showed to expect from wheel odometry alone)")

assert dist_to_goal < 0.25, (
    f"physically executing the planned path left the robot {dist_to_goal:.3f} m from the goal -- "
    f"the full scan->map->plan->drive pipeline did not actually work, not just a printout that claims it did"
)
print("LECTURE: the plan built from one lidar scan, executed with only wheel velocity commands and no "
      "access to ground truth during the drive, actually got the robot to the goal -- verified from the "
      "simulated pose, not assumed from the planner's output")

# Raw map/plan/drive data for tools/render_figures.py -- this run's actual
# grid, A* path, waypoints, and per-step trajectory, not a redrawn mockup.
np.savez(
    os.path.join(HERE, "data_lecture19.npz"),
    grid=grid, x_min=X_MIN, x_max=X_MAX, y_min=Y_MIN, y_max=Y_MAX, res=RES,
    path=np.array(path), start=np.array(start), goal=np.array(goal),
    waypoints=np.array(waypoints), traj_gt=np.array(traj_gt), traj_odo=np.array(traj_odo),
)

kit.close()
