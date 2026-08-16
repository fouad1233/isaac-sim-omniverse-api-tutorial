"""Lecture 13 -- Camera parameters deep dive.

Run it:
    <isaac-sim>/python.sh lectures/lecture13.py

Lecture 07 introduced focalLength and horizontalAperture just far enough to
derive horizontal FOV and confirm a cube renders dead center. This lecture
takes the same camera further: the intrinsics matrix those two numbers
(plus resolution) actually build, checked against where a known 3D point
really lands in the image -- and clippingRange, checked by making the same
cube disappear and reappear as near/far clip planes move through it.
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

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Same scene shape as Lecture 07: one red cube at the origin, a distant sun,
# a camera on the +Z axis with zero rotation -- local -Z already points
# straight down at the cube, no aiming math needed (Lecture 03 covered
# that; this lecture is about the lens, not where it's pointed).
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(1.0)
cube.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])

sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.XformCommonAPI(sun).SetRotate((-45.0, 0.0, 0.0))

CAM_Z = 5.0
CAM_PATH = "/World/Cam"
cam_geom = UsdGeom.Camera.Define(stage, CAM_PATH)
UsdGeom.XformCommonAPI(cam_geom).SetTranslate((0.0, 0.0, CAM_Z))

camera = Camera(prim_path=CAM_PATH, resolution=(320, 240))
camera.initialize()

timeline = omni.timeline.get_timeline_interface()
timeline.play()


def warm_up(max_steps: int = 40):
    """Same 'update() doesn't mean data is ready' loop Lecture 07 needed,
    run once at startup to find the first real frame."""
    for step in range(max_steps):
        kit.update()
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            return rgba
    raise RuntimeError("camera never produced a frame")


def render_after_change(settle_steps: int = 20):
    """Re-render after changing a camera parameter (clippingRange, here).

    The first non-empty get_rgba() after a change is NOT guaranteed to
    reflect that change -- an earlier version of this function returned as
    soon as it saw any frame, and every clippingRange test after the first
    silently showed the PREVIOUS setting's result, one step behind, because
    the render pipeline has its own buffering the same way Lecture 06/07's
    physics and rendering did. This runs a fixed number of update() calls
    and keeps only the LAST frame, discarding whatever was already in
    flight when the change was made.
    """
    rgba = None
    for _ in range(settle_steps):
        kit.update()
        new_rgba = camera.get_rgba()
        if new_rgba is not None and new_rgba.size > 0:
            rgba = new_rgba
    if rgba is None:
        raise RuntimeError("camera never produced a frame after the change")
    return rgba


def red_pixel_count_and_centroid(rgba):
    red_mask = (rgba[..., 0] > 150) & (rgba[..., 1] < 100) & (rgba[..., 2] < 100)
    ys, xs = np.where(red_mask)
    if len(xs) == 0:
        return 0, None
    return len(xs), (float(xs.mean()), float(ys.mean()))


rgba = warm_up()
n_red, centroid = red_pixel_count_and_centroid(rgba)
print(f"LECTURE: baseline render -- {n_red} red pixels, centroid={centroid}")

# =============================================================================
# Part 1: the intrinsics matrix -- built from four numbers you already have
# (focalLength, horizontal/verticalAperture, resolution), not something the
# API hands you as a mystery box.
# =============================================================================
focal_length = camera.get_focal_length()
h_aperture = camera.get_horizontal_aperture()
v_aperture = camera.get_vertical_aperture()
width, height = camera.get_resolution()
print(f"\nLECTURE: focalLength={focal_length}mm horizontalAperture={h_aperture:.3f}mm "
      f"verticalAperture={v_aperture:.3f}mm resolution={width}x{height}")

fx_hand = width * focal_length / h_aperture
fy_hand = height * focal_length / v_aperture
cx_hand, cy_hand = width * 0.5, height * 0.5
K_hand = np.array([[fx_hand, 0.0, cx_hand], [0.0, fy_hand, cy_hand], [0.0, 0.0, 1.0]])
K_api = np.array(camera.get_intrinsics_matrix())
print(f"LECTURE: hand-built intrinsics matrix:\n{K_hand}")
print(f"LECTURE: camera.get_intrinsics_matrix():\n{K_api}")
print(f"LECTURE: match = {np.allclose(K_hand, K_api)}")

# The real test of an intrinsics matrix isn't that it matches a formula --
# it's that it correctly predicts where a real 3D point lands on a real
# rendered image. The cube's center is a known world point: (0, 0, 0).
predicted = camera.get_image_coords_from_world_points(np.array([[0.0, 0.0, 0.0]]))[0]
print(f"\nLECTURE: predicted pixel coords of cube center (0,0,0) = "
      f"({predicted[0]:.1f}, {predicted[1]:.1f})")
print(f"LECTURE: measured red-pixel centroid                    = "
      f"({centroid[0]:.1f}, {centroid[1]:.1f})")
err = math.hypot(predicted[0] - centroid[0], predicted[1] - centroid[1])
print(f"LECTURE: prediction error = {err:.2f} px (should be small -- this is the "
      f"same projection every downstream perception module relies on)")

# =============================================================================
# Part 2: horizontal and vertical FOV are two different numbers, because
# horizontalAperture and verticalAperture are two different numbers -- a
# 320x240 (4:3) sensor is not square, and neither is its field of view.
# =============================================================================
computed_hfov_deg = 2 * math.degrees(math.atan(h_aperture / (2 * focal_length)))
computed_vfov_deg = 2 * math.degrees(math.atan(v_aperture / (2 * focal_length)))
reported_hfov_deg = math.degrees(camera.get_horizontal_fov())
reported_vfov_deg = math.degrees(camera.get_vertical_fov())
print(f"\nLECTURE: horizontal FOV -- trig formula={computed_hfov_deg:.4f}deg api={reported_hfov_deg:.4f}deg "
      f"(match: {abs(computed_hfov_deg - reported_hfov_deg) < 1e-3})")
print(f"LECTURE: vertical FOV   -- trig formula={computed_vfov_deg:.4f}deg api={reported_vfov_deg:.4f}deg "
      f"(match: {abs(computed_vfov_deg - reported_vfov_deg) < 1e-3})")
linear_scaled_deg = reported_hfov_deg * (height / width)
print(f"LECTURE: get_vertical_fov()'s actual formula is hfov * height/width, not "
      f"atan(verticalAperture/2*focalLength): {reported_hfov_deg:.4f} * {height}/{width} "
      f"= {linear_scaled_deg:.4f}deg -- matches the api value, not the trig formula")

# =============================================================================
# Part 3: clippingRange. The cube sits ~4.5-5.5m from a camera at z=5 (top
# face to bottom face). Narrow the far plane below that range and the cube
# should vanish; raise the near plane above it and it should vanish too --
# it's still exactly where it was, the renderer is just told not to draw
# anything in that depth band.
# =============================================================================
near0, far0 = camera.get_clipping_range()
print(f"\nLECTURE: default clippingRange = (near={near0}, far={far0})")

camera.set_clipping_range(near_distance=0.1, far_distance=20.0)
rgba = render_after_change()
n_red, _ = red_pixel_count_and_centroid(rgba)
print(f"LECTURE: clippingRange=(0.1, 20.0)  -- {n_red} red pixels (cube should be visible)")

camera.set_clipping_range(far_distance=4.0)
rgba = render_after_change()
n_red, _ = red_pixel_count_and_centroid(rgba)
print(f"LECTURE: clippingRange=(0.1, 4.0)   -- {n_red} red pixels "
      f"(far plane now in front of the cube -- should be 0)")

camera.set_clipping_range(near_distance=0.1, far_distance=20.0)
camera.set_clipping_range(near_distance=6.0)
rgba = render_after_change()
n_red, _ = red_pixel_count_and_centroid(rgba)
print(f"LECTURE: clippingRange=(6.0, 20.0)  -- {n_red} red pixels "
      f"(near plane now behind the cube from the camera's side -- should be 0)")

camera.set_clipping_range(near_distance=0.1, far_distance=20.0)
rgba = render_after_change()
n_red, _ = red_pixel_count_and_centroid(rgba)
print(f"LECTURE: clippingRange=(0.1, 20.0)  -- {n_red} red pixels "
      f"(restored -- confirms the cube itself was never the problem)")

kit.close()
