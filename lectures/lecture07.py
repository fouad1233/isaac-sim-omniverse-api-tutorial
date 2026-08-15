"""Lecture 07 -- Cameras and Rendering.

Run it:
    <isaac-sim>/python.sh lectures/lecture07.py
"""

import math
import os

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import UsdGeom, UsdLux  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PNG = os.path.join(HERE, "output_lecture07_top_down.png")


# --- Build a scene simple enough that "is the camera working" is the only
# question -- a red cube, a light, nothing else. ------------------------

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(1.0)
cube.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])  # plain red, no material needed

sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.XformCommonAPI(sun).SetRotate((-45.0, 0.0, 0.0))  # angled, not straight down

# A camera looking straight down at the cube from directly above. This is
# the one rotation that needs no thought at all: a camera's local -Z points
# down by default (Lecture 03), and world -Z is already "down" on this
# z-up stage, so placing it on the +Z axis with ZERO rotation already
# points it exactly at the cube. Aiming a camera at an angle is just
# Lecture 03's rotation math again -- this lecture is about capture, not
# aiming, so it isn't the thing being tested here.
CAM_PATH = "/World/Cam"
cam_geom = UsdGeom.Camera.Define(stage, CAM_PATH)
UsdGeom.XformCommonAPI(cam_geom).SetTranslate((0.0, 0.0, 5.0))

focal_length = cam_geom.GetFocalLengthAttr().Get()
horiz_aperture = cam_geom.GetHorizontalApertureAttr().Get()
print(f"LECTURE: UsdGeom.Camera defaults -- focalLength={focal_length} mm, "
      f"horizontalAperture={horiz_aperture:.3f} mm")

# --- The camera prim above is a USD schema: a position and a couple of
# lens numbers. It has no idea how to produce pixels -- that's a separate
# render pipeline, which isaacsim.sensors.camera.Camera wraps for you. ---

camera = Camera(prim_path=CAM_PATH, resolution=(320, 240))
camera.initialize()

timeline = omni.timeline.get_timeline_interface()
timeline.play()

first_valid_step = None
rgba = None
for step in range(40):
    kit.update()
    rgba = camera.get_rgba()
    if rgba is not None and rgba.size > 0 and first_valid_step is None:
        first_valid_step = step
        print(f"LECTURE: step {step} -- first step get_rgba() returned real "
              f"pixels (shape={rgba.shape})")
    if first_valid_step is not None and step >= first_valid_step + 3:
        break

if first_valid_step is None:
    raise RuntimeError("camera never produced a frame in 40 steps")
if first_valid_step > 0:
    print(f"LECTURE: steps 0..{first_valid_step - 1} all returned None/empty "
          f"-- the render pipeline wasn't ready yet, same 'update() doesn't "
          f"mean data is ready' lesson from Lecture 06, now for pixels "
          f"instead of physics.")

# --- Horizontal FOV is derived from the two lens numbers above, not stored
# separately -- worth confirming the formula rather than trusting it. ----

computed_fov_deg = 2 * math.degrees(math.atan(horiz_aperture / (2 * focal_length)))
reported_fov_rad = camera.get_horizontal_fov()
print(f"\nLECTURE: horizontal FOV from focalLength/aperture formula = "
      f"{computed_fov_deg:.4f} deg")
print(f"LECTURE: camera.get_horizontal_fov() = {reported_fov_rad:.4f} rad "
      f"= {math.degrees(reported_fov_rad):.4f} deg "
      f"(matches the formula: "
      f"{abs(math.degrees(reported_fov_rad) - computed_fov_deg) < 1e-3})")

# --- Save the frame and confirm the cube actually landed where geometry
# says it should: dead center, since the camera looks straight down at it.

from PIL import Image  # noqa: E402

Image.fromarray(rgba, mode="RGBA").save(OUTPUT_PNG)
print(f"\nLECTURE: saved {OUTPUT_PNG}")

red_mask = (rgba[..., 0] > 150) & (rgba[..., 1] < 100) & (rgba[..., 2] < 100)
ys, xs = np.where(red_mask)
h, w = rgba.shape[:2]
if len(xs) > 0:
    cx, cy = xs.mean(), ys.mean()
    print(f"LECTURE: red-pixel centroid=({cx:.1f}, {cy:.1f}), "
          f"image center=({w / 2:.1f}, {h / 2:.1f}) "
          f"-- {len(xs)} red pixels found")
else:
    print("LECTURE: no red pixels found -- something's wrong with the setup")

kit.close()
