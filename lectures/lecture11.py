"""Lecture 11 -- 2D LiDAR (RTX Lidar sensor, module 2).

Run it:
    <isaac-sim>/python.sh lectures/lecture11.py

This is the first lecture in the course that touches a content CDN: the
lidar "config" below (~15KB) is fetched from NVIDIA's asset server the
first time you run this, then cached locally. Everything else -- the room,
the scan, the verification -- is built and computed right here, same as
every other lecture.

Workstation-specific gotcha, fixed below and explained in lecture11.md:
this machine has two GPUs with no CUDA peer access. Isaac Sim's RTX
renderer defaults to spreading raytracing work across both, and the lidar
sensor's readback pipeline cannot cope with that split -- it comes back as
garbage ("invalid magic number") instead of real points. `multi_gpu=False,
active_gpu=0` below pins everything to one GPU and fixes it. If you're on a
single-GPU machine you likely don't need this, but it's harmless either way.
"""

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

captured = {"frames": []}


class CaptureWriter(Writer):
    """Grabs every decoded GenericModelOutput frame and hands them back out.

    The raw GMO buffer isn't decoded point data -- it's an opaque byte
    buffer. `parse_generic_model_output_data()` decodes it, and the only
    place you get to call that is inside a Writer's write() callback.

    A rotary lidar only covers part of its azimuth sweep per tick -- it's
    spinning, not flashing 360 degrees at once. Keeping only the newest
    tick's frame (overwriting a single `captured["gmo"]` each call, an
    earlier version of this script's mistake) gets you a scan stuck in a
    fixed, narrow azimuth window no matter how long you run it. Appending
    every tick's frame here is what actually accumulates a full rotation.
    """

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
            # gmo.x / .y / .z / .channelId are already numpy arrays (check the
            # type stub, not just the name) -- np.array(...) here just copies
            # them out before the underlying buffer gets reused next tick.
            captured["frames"].append({
                "x": np.array(gmo.x),
                "y": np.array(gmo.y),
                "z": np.array(gmo.z),
                "channelId": np.array(gmo.channelId),
            })
            captured["coords_type"] = gmo.elementsCoordsType


rep.WriterRegistry.register(CaptureWriter)


def build_room_and_scan(room_scale: float, num_ticks: int) -> dict:
    """Build a 4-wall sealed room at the given scale, scan it, return per-point data."""
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # A small, deliberately lopsided room -- footprint x in [-2,3], y in
    # [-4,3] -- so a real scan of it traces a recognizable, asymmetric shape
    # instead of a boring square. Walls fully span their side of that
    # rectangle (with a margin so corners overlap), plus a floor and ceiling,
    # so every ray from the sensor hits something real.
    cx, cy = 0.5, -0.5  # rectangle center
    half_x, half_y = 2.8, 3.8  # half-width/depth, with margin past the true 2.5/3.5
    wall_specs = [
        ("North", (cx, 3.0, 1.0), (2 * half_x, 0.2, 2.0)),
        ("South", (cx, -4.0, 1.0), (2 * half_x, 0.2, 2.0)),
        ("East", (3.0, cy, 1.0), (0.2, 2 * half_y, 2.0)),
        ("West", (-2.0, cy, 1.0), (0.2, 2 * half_y, 2.0)),
        ("Floor", (cx, cy, 0.0), (2 * half_x, 2 * half_y, 0.2)),
        ("Ceiling", (cx, cy, 2.0), (2 * half_x, 2 * half_y, 0.2)),
    ]
    for name, pos, scale in wall_specs:
        spos = tuple(p * room_scale for p in pos)
        sscale = tuple(s * room_scale for s in scale)
        wall = UsdGeom.Cube.Define(stage, f"/World/Wall{name}")
        wall.CreateSizeAttr(1.0)
        UsdGeom.XformCommonAPI(wall).SetTranslate(spos)
        UsdGeom.XformCommonAPI(wall).SetScale(sscale)
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

    lidar = Lidar.create(
        "/World/Lidar",
        config="Example_Rotary_2D",
        translations=np.array([0.0, 0.0, 1.0 * room_scale]),
        aux_output_level="BASIC",
    )
    if room_scale == 1.0:
        prim = stage.GetPrimAtPath(lidar.paths[0])
        nchan = prim.GetAttribute("omni:sensor:Core:numberOfChannels").Get()
        print(f"LECTURE: numberOfChannels attribute = {nchan}")

    sensor = LidarSensor(lidar, annotators=[])
    captured["frames"] = []
    captured["coords_type"] = None
    sensor.attach_writer("CaptureWriter")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(num_ticks):
        kit.update()
    timeline.stop()

    frames = captured["frames"]
    if not frames:
        print(f"LECTURE: scale={room_scale}: no lidar data captured -- something is wrong, see lecture11.md")
        kit.close()
        raise SystemExit(1)

    raw_x = np.concatenate([f["x"] for f in frames])
    raw_y = np.concatenate([f["y"] for f in frames])
    raw_z = np.concatenate([f["z"] for f in frames])
    ch = np.concatenate([f["channelId"] for f in frames])
    n = len(raw_x)

    # THE finding of this lecture: GenericModelOutput's x/y/z are NOT
    # cartesian by default. `elementsCoordsType` says so -- check it, don't
    # assume it -- and here it reports SPHERICAL, which per NVIDIA's own
    # gmo_lib docs means:
    #   x = azimuth (degrees, [-180, 180])
    #   y = elevation (degrees, [-90, 90])
    #   z = distance (meters)
    # Feed these three numbers into sqrt(x^2+y^2+z^2) and atan2() like they
    # were cartesian -- which is exactly what an earlier version of this
    # script did -- and you get numbers that *look* like a plausible lidar
    # reading (distances of tens of meters, elevations up to 90 degrees) but
    # correspond to nothing physical at all. Nothing throws. Nothing warns.
    # It just quietly computes garbage from real numbers.
    coords_type = captured["coords_type"]
    if coords_type != CoordsType.SPHERICAL:
        print(f"LECTURE: elementsCoordsType = {coords_type!r}, not SPHERICAL -- "
              "this script's field mapping (x=az, y=el, z=dist) does not apply here, "
              "see lecture11.md for what CARTESIAN mode means instead")
        kit.close()
        raise SystemExit(1)
    az_deg, el_deg, dist_m = raw_x, raw_y, raw_z

    per_channel_dist = {}
    for c in np.unique(ch):
        c = int(c)
        per_channel_dist[c] = float(np.median(dist_m[ch == c]))

    return {
        "n": n, "az_deg": az_deg, "el_deg": el_deg, "dist_m": dist_m,
        "channel_id": ch, "per_channel_dist": per_channel_dist, "coords_type": coords_type,
    }


# =============================================================================
# Step 1: don't assume the coordinate convention -- read it.
# =============================================================================
print("LECTURE: scanning the room at its authored scale (1x)...")
scan1 = build_room_and_scan(1.0, num_ticks=200)
print(f"LECTURE:   elementsCoordsType = {scan1['coords_type']!r} -- x/y/z below are read as (azimuth, elevation, distance)")
print(f"LECTURE:   captured {scan1['n']} beam returns")
print(f"LECTURE:   elevation range = [{scan1['el_deg'].min():.2f}, {scan1['el_deg'].max():.2f}] deg "
      f"-- this config really is single-ring 2D (elevation ~= 0 for every point)"
      if abs(scan1["el_deg"].max() - scan1["el_deg"].min()) < 1.0 else
      f"LECTURE:   elevation range = [{scan1['el_deg'].min():.2f}, {scan1['el_deg'].max():.2f}] deg "
      f"-- NOT single-ring, multiple elevations present despite the '_2D' name")
print(f"LECTURE:   azimuth coverage = [{scan1['az_deg'].min():.1f}, {scan1['az_deg'].max():.1f}] deg "
      f"over {len(np.unique(scan1['channel_id']))} distinct channel IDs")

# =============================================================================
# Step 2: verify anyway. Scale the SAME room 4x and check that distance
# scales with it. This isn't rescuing broken data anymore (that was the
# cartesian-on-spherical bug) -- it's a real safety check for the next
# lecture and for your own scenes: if a "hit" doesn't move when the wall
# does, something in your setup is still wrong.
# =============================================================================
SCALE = 4.0
print(f"\nLECTURE: rebuilding the same room {SCALE}x bigger and scanning again...")
scan2 = build_room_and_scan(SCALE, num_ticks=200)
print(f"LECTURE:   captured {scan2['n']} beam returns")

trustworthy = []
for c in sorted(scan1["per_channel_dist"]):
    if c not in scan2["per_channel_dist"]:
        continue
    d1, d2 = scan1["per_channel_dist"][c], scan2["per_channel_dist"][c]
    ratio = d2 / d1 if d1 > 0 else float("nan")
    if abs(ratio - SCALE) < 0.5:
        trustworthy.append(c)

n_common = sum(1 for c in scan1["per_channel_dist"] if c in scan2["per_channel_dist"])
print(f"\nLECTURE: {len(trustworthy)}/{n_common} channels scaled distance by ~{SCALE}x as expected "
      f"when the room grew {SCALE}x.")

# =============================================================================
# Step 3: the actual 2D scan, straight from the correctly-interpreted fields
# -- no trigonometry required, because azimuth and distance were already
# sitting right there in the "x" and "z" arrays the whole time.
# =============================================================================
az_deg, dist_m = scan1["az_deg"], scan1["dist_m"]
print("\nLECTURE: 2D scan sampled every 45deg (nearest point, within 10deg):")
for target_az in range(-180, 180, 45):
    delta = np.abs(((az_deg - target_az + 180) % 360) - 180)
    nearby = delta < 10.0
    if nearby.any():
        d = dist_m[nearby][np.argmin(dist_m[nearby])]
        print(f"LECTURE:   azimuth {target_az:5d}deg -> {d:.2f} m")
    else:
        print(f"LECTURE:   azimuth {target_az:5d}deg -> no return nearby")
print("LECTURE: compare against the room: East(az 0)=3m North(az 90)=3m West(az +-180)=2m South(az -90)=4m")

# Raw scan data for tools/render_figures.py -- a top-down scatter of the
# real decoded returns, not a redrawn illustration of what the room
# "should" look like.
np.savez(os.path.join(HERE, "data_lecture11.npz"), az_deg=az_deg, dist_m=dist_m)

kit.close()
