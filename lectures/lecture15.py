"""Lecture 15 -- Mapping (occupancy grid).

Run it:
    <isaac-sim>/python.sh lectures/lecture15.py

Lecture 11 scanned this exact sealed room and, for each of 8 compass
directions, printed the single nearest verified distance. An occupancy
grid is the same idea taken all the way: bin every azimuth finely, keep
the nearest return in each bin (the wall really there, not something
behind it), then for every cell in a 2D grid compare its own distance from
the sensor to that direction's measured range -- nearer means free space
the beam passed through, at-range means the surface that stopped it,
farther means unobserved, behind whatever was actually hit.
"""

import numpy as np
from isaacsim import SimulationApp

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

# The exact sealed room from Lecture 11 -- x in [-2,3], y in [-4,3], floor
# and ceiling included. Reused byte-for-byte on purpose: this lecture's
# job is turning Lecture 11's verified (azimuth, distance) data into a
# grid, not re-verifying the sensor.
cx, cy = 0.5, -0.5
half_x, half_y = 2.8, 3.8
wall_specs = [
    ("North", (cx, 3.0, 1.0), (2 * half_x, 0.2, 2.0)),
    ("South", (cx, -4.0, 1.0), (2 * half_x, 0.2, 2.0)),
    ("East", (3.0, cy, 1.0), (0.2, 2 * half_y, 2.0)),
    ("West", (-2.0, cy, 1.0), (0.2, 2 * half_y, 2.0)),
    ("Floor", (cx, cy, 0.0), (2 * half_x, 2 * half_y, 0.2)),
    ("Ceiling", (cx, cy, 2.0), (2 * half_x, 2 * half_y, 0.2)),
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
    """Same as Lecture 11's -- accumulate every tick, copy gmo.x/.y/.z out
    as numpy arrays (already numpy per the .pyi stub), track elementsCoordsType."""

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
# Step 1: bin azimuth finely and keep the NEAREST return per bin. A wall
# can't be seen through, so of everything a bin's beams hit, the closest
# one is the real surface -- everything past it is occlusion, not
# additional geometry.
# =============================================================================
N_BINS = 720  # 0.5deg per bin
bin_edges = np.linspace(-180.0, 180.0, N_BINS + 1)
bin_idx = np.clip(np.digitize(az_deg, bin_edges) - 1, 0, N_BINS - 1)
bin_range = np.full(N_BINS, np.inf)
np.minimum.at(bin_range, bin_idx, dist_m)
n_empty_bins = int(np.isinf(bin_range).sum())
print(f"LECTURE: {N_BINS} azimuth bins ({360 / N_BINS:.2f}deg each), {n_empty_bins} received no return")

# =============================================================================
# Step 2: rasterize. For every grid cell, find its own (r, theta) from the
# sensor at the origin, look up that direction's measured range, and
# compare -- vectorized over the whole grid at once, no per-point ray
# marching needed.
# =============================================================================
RES = 0.2  # meters per cell
X_MIN, X_MAX = -3.0, 4.0
Y_MIN, Y_MAX = -5.0, 4.0
xs = np.arange(X_MIN, X_MAX, RES)
ys = np.arange(Y_MIN, Y_MAX, RES)
grid_x, grid_y = np.meshgrid(xs + RES / 2, ys + RES / 2)  # cell centers

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
      f"{n_free} free, {n_occ} occupied, {n_unk} unknown")

# =============================================================================
# Step 3: verify against the known room, same expected values Lecture 11
# checked (sensor at world (0,0), room center at (0.5,-0.5) -- East/North
# expected ~2.9m, South ~3.9m, West ~1.9m).
# =============================================================================


def nearest_occupied_range(target_az_deg: float) -> float:
    b = np.clip(np.digitize([target_az_deg], bin_edges)[0] - 1, 0, N_BINS - 1)
    return float(bin_range[b])


for label, az in [("East (0deg)", 0.0), ("North (90deg)", 90.0), ("West (180deg)", 180.0), ("South (-90deg)", -90.0)]:
    print(f"LECTURE:   {label:16s} nearest occupied range = {nearest_occupied_range(az):.2f} m")

# =============================================================================
# Step 4: print it. North at the top, East to the right -- a standard
# top-down map orientation, built entirely from one static scan.
# =============================================================================
CHARS = {UNKNOWN: " ", FREE: ".", OCCUPIED: "#"}
print(f"\nLECTURE: occupancy grid ('#'=occupied '.'=free ' '=unknown), "
      f"x:[{X_MIN},{X_MAX}] y:[{Y_MIN},{Y_MAX}] at {RES}m/cell:")
for row in range(grid.shape[0] - 1, -1, -1):
    print("LECTURE: " + "".join(CHARS[v] for v in grid[row]))

kit.close()
