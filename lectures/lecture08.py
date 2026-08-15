"""Lecture 08 -- Lights and Why Dome Lights Don't Reach Interiors.

Run it:
    <isaac-sim>/python.sh lectures/lecture08.py
"""

import os

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import UsdGeom, UsdLux  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def add_wall(stage, name, center, size, color=(0.8, 0.8, 0.8)):
    """A cube's `size` attribute is a single uniform edge length -- there's
    no separate width/depth/height on the schema. A wall needs those three
    to differ, so this builds one the way USD expects: a unit cube (size=1,
    spanning -0.5..0.5 on every axis) plus a non-uniform SetScale to stretch
    it, plus SetTranslate to place its center. Same XformCommonAPI Lecture
    03 already covered, just with the third op it didn't need until now.
    """
    cube = UsdGeom.Cube.Define(stage, f"/World/{name}")
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([color])
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetScale(size)
    xf.SetTranslate(center)
    return cube


def build_room(stage):
    """A fully enclosed box: floor, ceiling, four walls, no gaps. A cube
    sits in the middle as something for light to actually land on.
    """
    add_wall(stage, "Floor", (0, 0, 0.0), (6, 4, 0.1))
    add_wall(stage, "Ceiling", (0, 0, 3.0), (6, 4, 0.1))
    add_wall(stage, "WallFar", (3.0, 0, 1.5), (0.1, 4, 3))
    add_wall(stage, "WallNear", (-3.0, 0, 1.5), (0.1, 4, 3))
    add_wall(stage, "WallLeft", (0, 2.0, 1.5), (6, 0.1, 3))
    add_wall(stage, "WallRight", (0, -2.0, 1.5), (6, 0.1, 3))

    target = UsdGeom.Cube.Define(stage, "/World/TargetCube")
    target.CreateSizeAttr(0.8)
    target.CreateDisplayColorAttr([(0.9, 0.9, 0.9)])
    UsdGeom.XformCommonAPI(target).SetTranslate((0.5, 0.0, 0.9))
    return target


def build_camera(stage):
    """Same (0, -90, 0) rotation Lecture 03 verified and Lecture 07's
    top-down shot deliberately avoided needing -- this is the case where
    it's actually necessary: looking down the room's length, world +X.
    """
    cam_geom = UsdGeom.Camera.Define(stage, "/World/Cam")
    xf = UsdGeom.XformCommonAPI(cam_geom)
    xf.SetTranslate((-2.5, 0.0, 1.5))
    xf.SetRotate((0.0, -90.0, 0.0))
    return cam_geom


def capture(camera, timeline, settle_steps=10):
    timeline.play()
    rgba = None
    for _ in range(settle_steps):
        kit.update()
        candidate = camera.get_rgba()
        if candidate is not None and candidate.size > 0:
            rgba = candidate
    timeline.stop()
    kit.update()
    return rgba


def mean_brightness(rgba):
    return float(rgba[..., :3].astype(np.float32).mean())


results = {}

# --- Case A: dome light only, ordinary intensity -----------------------
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
build_room(stage)
cam_geom = build_camera(stage)
dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
dome.CreateIntensityAttr(1000.0)

camera = Camera(prim_path="/World/Cam", resolution=(320, 240))
camera.initialize()
timeline = omni.timeline.get_timeline_interface()

rgba_a = capture(camera, timeline)
results["A_dome_1000"] = mean_brightness(rgba_a)
Image.fromarray(rgba_a, mode="RGBA").save(os.path.join(HERE, "output_lecture08_a_dome_only.png"))

# --- Case B: same dome light, intensity turned up 50x -------------------
dome.GetIntensityAttr().Set(50000.0)
rgba_b = capture(camera, timeline)
results["B_dome_50000"] = mean_brightness(rgba_b)
Image.fromarray(rgba_b, mode="RGBA").save(os.path.join(HERE, "output_lecture08_b_dome_bright.png"))

# --- Case C: dome back to normal, PLUS a modest light placed inside -----
dome.GetIntensityAttr().Set(1000.0)
interior = UsdLux.SphereLight.Define(stage, "/World/InteriorLight")
interior.CreateRadiusAttr(0.15)
interior.CreateIntensityAttr(15000.0)
UsdGeom.XformCommonAPI(interior).SetTranslate((-1.5, 0.0, 2.7))

rgba_c = capture(camera, timeline)
results["C_dome_1000_plus_interior"] = mean_brightness(rgba_c)
Image.fromarray(rgba_c, mode="RGBA").save(os.path.join(HERE, "output_lecture08_c_interior_light.png"))

print("LECTURE: mean RGB brightness (0-255 scale) seen by a camera INSIDE the sealed room")
for key, value in results.items():
    print(f"LECTURE:   {key:32s} -> {value:6.2f}")

print(f"\nLECTURE: 50x brighter dome changed interior brightness by "
      f"{results['B_dome_50000'] - results['A_dome_1000']:+.2f} "
      f"(barely, if at all -- intensity wasn't the problem)")
print(f"LECTURE: one modest interior light changed it by "
      f"{results['C_dome_1000_plus_interior'] - results['A_dome_1000']:+.2f} "
      f"(placement was)")

kit.close()
