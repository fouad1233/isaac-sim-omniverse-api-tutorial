"""Lecture 14 -- Depth cameras and RGB-D.

Run it:
    <isaac-sim>/python.sh lectures/lecture14.py

Two different "depth" annotators exist on the same camera:
`distance_to_image_plane` (Z along the optical axis -- what RGB-D sensors
and depth backprojection formulas mean by "depth") and `distance_to_camera`
(true Euclidean range from the camera's origin -- what a lidar reports).
They are NOT the same number off-center, and this lecture measures exactly
how they differ, using the same intrinsics matrix Lecture 13 built and
verified.
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

# A large, flat floor -- big enough to fill the whole frame at this camera's
# height and FOV, so every pixel hits the SAME perpendicular surface at the
# SAME known height. That's what makes the two depth conventions cleanly
# separable: distance_to_image_plane should come out constant across the
# whole image (it's just camera height), distance_to_camera should not (it
# grows toward the image edges, purely from off-axis geometry).
floor = UsdGeom.Cube.Define(stage, "/World/Floor")
floor.CreateSizeAttr(1.0)
floor.CreateDisplayColorAttr([(0.6, 0.6, 0.6)])
UsdGeom.XformCommonAPI(floor).SetTranslate((0.0, 0.0, -0.05))
UsdGeom.XformCommonAPI(floor).SetScale((10.0, 10.0, 0.1))  # top face at z=0, spans x,y in [-5,5]

sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.XformCommonAPI(sun).SetRotate((-45.0, 0.0, 0.0))

CAM_Z = 5.0
CAM_PATH = "/World/Cam"
cam_geom = UsdGeom.Camera.Define(stage, CAM_PATH)
UsdGeom.XformCommonAPI(cam_geom).SetTranslate((0.0, 0.0, CAM_Z))

camera = Camera(prim_path=CAM_PATH, resolution=(320, 240))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_distance_to_camera_to_frame()

timeline = omni.timeline.get_timeline_interface()
timeline.play()


def settle(steps: int = 30) -> dict:
    """Lecture 13's fix, reused: keep updating, keep only the LAST frame --
    the first available frame after annotators are attached is no more
    trustworthy here than it was for a changed clippingRange."""
    frame = None
    for _ in range(steps):
        kit.update()
        f = camera.get_current_frame()
        if f.get("rgb") is not None and getattr(f["rgb"], "size", 0) > 0 \
                and f.get("distance_to_image_plane") is not None \
                and f.get("distance_to_camera") is not None:
            frame = f
    if frame is None:
        raise RuntimeError("camera never produced a complete frame")
    return frame


frame = settle()
depth_plane = np.array(frame["distance_to_image_plane"]).squeeze()
depth_cam = np.array(frame["distance_to_camera"]).squeeze()
print(f"LECTURE: distance_to_image_plane shape={depth_plane.shape} dtype={depth_plane.dtype}")
print(f"LECTURE: distance_to_camera      shape={depth_cam.shape} dtype={depth_cam.dtype}")

# =============================================================================
# Part 1: distance_to_image_plane should be almost exactly constant -- every
# pixel looks at the same flat floor, CAM_Z meters straight down.
# =============================================================================
print(f"\nLECTURE: distance_to_image_plane -- min={depth_plane.min():.5f} "
      f"max={depth_plane.max():.5f} mean={depth_plane.mean():.5f} "
      f"(camera height = {CAM_Z})")

# =============================================================================
# Part 2: distance_to_camera is NOT constant -- it grows toward the edges of
# the frame, and by exactly how much is predictable from the intrinsics
# matrix Lecture 13 built: for a pixel at normalized offset (x, y) =
# ((u-cx)/fx, (v-cy)/fy) from the principal point, the ray direction is
# (x, y, 1) in camera space, and distance_to_camera / distance_to_image_plane
# equals that ray's length, sqrt(x^2 + y^2 + 1).
# =============================================================================
focal_length = camera.get_focal_length()
h_aperture = camera.get_horizontal_aperture()
v_aperture = camera.get_vertical_aperture()
width, height = camera.get_resolution()
fx = width * focal_length / h_aperture
fy = height * focal_length / v_aperture
cx, cy = width * 0.5, height * 0.5

vs, us = np.mgrid[0:depth_plane.shape[0], 0:depth_plane.shape[1]]
# depth_plane's first axis -- confirm empirically whether it's rows (v) or
# columns (u) by checking shape against (height, width) printed above rather
# than assuming; mgrid above matches shape order directly either way.
x_norm = (us - cx) / fx
y_norm = (vs - cy) / fy
predicted_ratio = np.sqrt(x_norm**2 + y_norm**2 + 1.0)
measured_ratio = depth_cam / depth_plane
max_err = np.abs(predicted_ratio - measured_ratio).max()
print(f"\nLECTURE: distance_to_camera / distance_to_image_plane vs sqrt(x^2+y^2+1) prediction: "
      f"max abs error = {max_err:.6f}")
print(f"LECTURE: center pixel ratio = {measured_ratio[int(cy), int(cx)]:.5f} (should be ~1.0)")
corner_ratio = measured_ratio[0, 0]
print(f"LECTURE: corner pixel ratio = {corner_ratio:.5f} "
      f"(should be > 1.0 -- the corner ray is longer than the straight-down one)")

# =============================================================================
# Part 3: backprojection. get_world_points_from_image_coords() expects
# distance_to_image_plane specifically, not distance_to_camera -- its
# formula is inv(K) @ [u, v, 1]^T * depth, which only recovers the correct
# (X, Y, Z) point if depth == Z. Feed it distance_to_camera instead and
# every backprojected point would be wrong by exactly the ratio measured
# above. Checked here on 5 pixels: center and all four corners, each should
# backproject to a world point at height z=0 -- the floor's real height --
# regardless of which pixel it came from.
# =============================================================================
h_px, w_px = depth_plane.shape
sample_px = np.array([
    [cx, cy], [0, 0], [w_px - 1, 0], [0, h_px - 1], [w_px - 1, h_px - 1],
], dtype=np.float32)
sample_depth = np.array([depth_plane[int(v), int(u)] for u, v in sample_px], dtype=np.float32)
world_pts = camera.get_world_points_from_image_coords(sample_px, sample_depth)
print("\nLECTURE: backprojected world points (pixel -> world XYZ), floor is at z=0:")
labels = ["center", "top-left", "top-right", "bottom-left", "bottom-right"]
for label, px, wp in zip(labels, sample_px, np.array(world_pts)):
    print(f"LECTURE:   {label:12s} pixel=({px[0]:.0f},{px[1]:.0f}) -> world=({wp[0]:+.3f}, {wp[1]:+.3f}, {wp[2]:+.3f})")

np.savez(
    os.path.join(HERE, "data_lecture14.npz"),
    depth_plane=depth_plane,
    depth_cam=depth_cam,
)

kit.close()
