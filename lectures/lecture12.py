"""Lecture 12 -- 3D LiDAR (RTX Lidar sensor, module 2).

Run it:
    <isaac-sim>/python.sh lectures/lecture12.py

Same GenericModelOutput decoding pipeline as Lecture 11, same
elementsCoordsType check, same multi_gpu=False workaround for this
workstation's no-P2P dual GPUs (see lecture11.md if you haven't read it --
this lecture assumes it). The only real change is the sensor config:
"Example_Rotary" instead of "Example_Rotary_2D" -- a config whose emitters
genuinely span a range of elevation angles, not one flat ring. Lecture 11's
"elevation ~= 0 for every point" finding was correct for that sensor; this
lecture exists to show it was never a general truth about the API, only a
fact about that one config.
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

captured = {"frames": [], "coords_type": None}


class CaptureWriter(Writer):
    """Identical to Lecture 11's -- accumulates every tick's decoded frame,
    copies gmo.x/.y/.z/.channelId out as numpy arrays (they already are
    numpy arrays per the .pyi stub; np.array(...) just copies before the
    buffer gets reused), and records elementsCoordsType once."""

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
            captured["frames"].append({
                "x": np.array(gmo.x),
                "y": np.array(gmo.y),
                "z": np.array(gmo.z),
                "channelId": np.array(gmo.channelId),
            })
            captured["coords_type"] = gmo.elementsCoordsType


rep.WriterRegistry.register(CaptureWriter)


def build_room_and_scan(room_scale: float, num_ticks: int) -> dict:
    """Same sealed room as Lecture 11 -- x in [-2,3], y in [-4,3], floor at
    z=0, ceiling at z=2, all times room_scale -- so a 3D sensor mounted at
    z=1*room_scale has floor and ceiling in range as well as all four
    walls, not just the walls Lecture 11's flat scan ever reached."""
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

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
        spos = tuple(p * room_scale for p in pos)
        sscale = tuple(s * room_scale for s in scale)
        wall = UsdGeom.Cube.Define(stage, f"/World/Wall{name}")
        wall.CreateSizeAttr(1.0)
        UsdGeom.XformCommonAPI(wall).SetTranslate(spos)
        UsdGeom.XformCommonAPI(wall).SetScale(sscale)
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

    lidar = Lidar.create(
        "/World/Lidar",
        config="Example_Rotary",
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
        print(f"LECTURE: scale={room_scale}: no lidar data captured -- something is wrong, see lecture12.md")
        kit.close()
        raise SystemExit(1)

    raw_x = np.concatenate([f["x"] for f in frames])
    raw_y = np.concatenate([f["y"] for f in frames])
    raw_z = np.concatenate([f["z"] for f in frames])
    ch = np.concatenate([f["channelId"] for f in frames])
    n = len(raw_x)

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
# Step 1: read the elevation spread instead of assuming it -- Lecture 11's
# "~= 0 for every point" was a fact about that config, not a fact about
# this API.
# =============================================================================
print("LECTURE: scanning the room at its authored scale (1x)...")
scan1 = build_room_and_scan(1.0, num_ticks=200)
print(f"LECTURE:   elementsCoordsType = {scan1['coords_type']!r} -- x/y/z below are read as (azimuth, elevation, distance)")
print(f"LECTURE:   captured {scan1['n']} beam returns")
uniq_el = np.unique(np.round(scan1["el_deg"], 2))
print(f"LECTURE:   elevation range = [{uniq_el.min():.2f}, {uniq_el.max():.2f}] deg "
      f"across {len(uniq_el)} distinct elevation rings -- this one is genuinely 3D")
print(f"LECTURE:   azimuth coverage = [{scan1['az_deg'].min():.1f}, {scan1['az_deg'].max():.1f}] deg "
      f"over {len(np.unique(scan1['channel_id']))} distinct channel IDs")
print(f"LECTURE:   distance range = [{scan1['dist_m'].min():.2f}, {scan1['dist_m'].max():.2f}] m "
      f"(config farRangeM=200 -- nothing near that means every ray hit the sealed room, none escaped)")

# =============================================================================
# Step 2: the same scale-verification check from Lecture 11, unchanged.
# It's not a coordinate-bug workaround -- it's a general check that a
# reported hit distance actually tracks the geometry it's measuring, and it
# applies just as well to floor/ceiling hits from a real 3D sensor as it did
# to Lecture 11's single azimuth ring of wall hits.
# =============================================================================
SCALE = 4.0
print(f"\nLECTURE: rebuilding the same room {SCALE}x bigger (mount height scales too) and scanning again...")
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
print(f"LECTURE: {len(trustworthy)}/{n_common} channels scaled distance by ~{SCALE}x as expected "
      f"when the room (and mount height) grew {SCALE}x.")

# =============================================================================
# Step 3: a vertical profile -- median distance per elevation ring, at the
# 1x scale. Don't assume what shape this takes; print it and look. It comes
# out as a gentle U -- shortest near 0deg (looking level), a little longer
# at the extremes -- and the reason is that this room's walls are full
# floor-to-ceiling height (see wall_specs: z-scale 2.0*room_scale, exactly
# matching the floor/ceiling gap), so a ray at ANY elevation in this
# sensor's +-15/+10deg range hits a wall, never the floor or ceiling. The
# only thing elevation changes is the slant distance to that same wall:
# distance(e) = distance(0deg) / cos(e). See lecture12.md for that formula
# checked against these exact printed numbers, to a few millimeters.
# =============================================================================
az_deg, el_deg, dist_m = scan1["az_deg"], scan1["el_deg"], scan1["dist_m"]
print(f"\nLECTURE: vertical profile -- median distance per elevation ring ({len(uniq_el)} rings, sampled every few):")
for e in uniq_el[::4]:
    in_ring = np.abs(el_deg - e) < 0.05
    if in_ring.any():
        d = np.median(dist_m[in_ring])
        print(f"LECTURE:   elevation {e:6.2f}deg -> median distance {d:5.2f} m over {int(in_ring.sum())} returns")

# Raw scan data for tools/render_figures.py -- a real decoded 3D point
# cloud (az/el/dist, spherical per elementsCoordsType), not a mockup.
np.savez(
    os.path.join(HERE, "data_lecture12.npz"),
    az_deg=az_deg, el_deg=el_deg, dist_m=dist_m, channel_id=scan1["channel_id"],
)

kit.close()
