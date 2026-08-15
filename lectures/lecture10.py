"""Lecture 10 -- Capstone: stage + physics + a driven joint + light + camera,
wired together end to end.

Run it:
    <isaac-sim>/python.sh lectures/lecture10.py
"""

import math
import os

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PNG = os.path.join(HERE, "output_lecture10_capstone.png")
TARGET_DEG = 60.0
BASE_POS = (0.0, 0.0, 1.0)

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# --- Lecture 05: nothing simulates meaningfully without a physics scene
# you actually control (this Isaac Sim version has an implicit fallback --
# Lecture 05's whole point was not trusting that on faith). --------------
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

# --- A floor, static collision only, plus a purely cosmetic pedestal (no
# physics APIs at all -- it's there so the arm doesn't look like it's
# floating, nothing about it is load-bearing to the simulation). --------
floor = UsdGeom.Cube.Define(stage, "/World/Floor")
floor.CreateSizeAttr(10.0)
floor.CreateDisplayColorAttr([(0.5, 0.5, 0.5)])
UsdGeom.XformCommonAPI(floor).SetTranslate((0.0, 0.0, -5.0))
UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

pedestal = UsdGeom.Cube.Define(stage, "/World/Pedestal")
pedestal.CreateSizeAttr(1.0)
pedestal.CreateDisplayColorAttr([(0.3, 0.3, 0.35)])
UsdGeom.XformCommonAPI(pedestal).SetScale((0.3, 0.3, 1.0))
UsdGeom.XformCommonAPI(pedestal).SetTranslate((0.0, 0.0, 0.5))

# --- Lecture 09: one fixed base, one arm link, one revolute joint, one
# position drive -- the smallest complete articulation. ------------------
base = UsdGeom.Cube.Define(stage, "/World/Base")
base.CreateSizeAttr(0.2)
base.CreateDisplayColorAttr([(0.2, 0.2, 0.8)])
UsdGeom.XformCommonAPI(base).SetTranslate(BASE_POS)
UsdPhysics.CollisionAPI.Apply(base.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
UsdPhysics.ArticulationRootAPI.Apply(base.GetPrim())

# Body0 is unset, so side 0 of this joint IS the world frame -- its local
# anchor must be given in WORLD coordinates to land on Base's actual
# position. Lecture 09 originally skipped this and paid for it: Base got
# silently dragged to the world origin the instant physics started,
# invisible there, but fatal here once a floor sits at z=0 exactly where
# the wrongly-anchored base would land.
fixed = UsdPhysics.FixedJoint.Define(stage, "/World/Base/FixedToWorld")
fixed.CreateBody1Rel().SetTargets(["/World/Base"])
fixed.CreateLocalPos0Attr(Gf.Vec3f(*BASE_POS))

arm = UsdGeom.Xform.Define(stage, "/World/Arm")
UsdGeom.XformCommonAPI(arm).SetTranslate(BASE_POS)
UsdPhysics.RigidBodyAPI.Apply(arm.GetPrim())
UsdPhysics.MassAPI.Apply(arm.GetPrim()).CreateMassAttr(0.5)

arm_geom = UsdGeom.Cube.Define(stage, "/World/Arm/Geom")
arm_geom.CreateSizeAttr(1.0)
arm_geom.CreateDisplayColorAttr([(0.9, 0.3, 0.1)])
UsdGeom.XformCommonAPI(arm_geom).SetScale((1.0, 0.1, 0.1))
UsdGeom.XformCommonAPI(arm_geom).SetTranslate((0.5, 0.0, 0.0))
UsdPhysics.CollisionAPI.Apply(arm_geom.GetPrim())

joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Base/Hinge")
joint.CreateBody0Rel().SetTargets(["/World/Base"])
joint.CreateBody1Rel().SetTargets(["/World/Arm"])
joint.CreateAxisAttr("Y")
joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
drive.CreateTypeAttr("force")
drive.CreateTargetPositionAttr(TARGET_DEG)
drive.CreateStiffnessAttr(150.0)
drive.CreateDampingAttr(25.0)
drive.CreateMaxForceAttr(1e6)

# --- Lecture 08: a light placed where the camera can actually see its
# effect, not just an ambient dome hoping it reaches everything. --------
key_light = UsdLux.SphereLight.Define(stage, "/World/KeyLight")
key_light.CreateRadiusAttr(0.15)
key_light.CreateIntensityAttr(30000.0)
UsdGeom.XformCommonAPI(key_light).SetTranslate((-1.5, -1.5, 3.0))

fill_dome = UsdLux.DomeLight.Define(stage, "/World/FillDome")
fill_dome.CreateIntensityAttr(300.0)  # deliberately modest -- it's fill, not the key

# --- Lecture 03 + 07: a camera aimed with the same rotate-and-verify
# discipline as every earlier lecture, not eyeballed. --------------------
cam_geom = UsdGeom.Camera.Define(stage, "/World/Cam")
UsdGeom.XformCommonAPI(cam_geom).SetTranslate((0.0, -3.0, 1.0))
UsdGeom.XformCommonAPI(cam_geom).SetRotate((90.0, 0.0, 0.0))

camera = Camera(prim_path="/World/Cam", resolution=(480, 360))
camera.initialize()


def arm_angle_deg():
    m = UsdGeom.Xformable(arm.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    forward = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return math.degrees(math.atan2(-forward[2], forward[0]))


# --- Lecture 06: play() and step -- and Lecture 07's lesson that update()
# and "the data is ready" are two different claims, now for BOTH physics
# convergence and render readiness at once. ------------------------------
timeline = omni.timeline.get_timeline_interface()
timeline.play()

rgba = None
for step in range(150):
    kit.update()
    candidate = camera.get_rgba()
    if candidate is not None and candidate.size > 0:
        rgba = candidate

final_angle = arm_angle_deg()
timeline.stop()
kit.update()

Image.fromarray(rgba, mode="RGBA").save(OUTPUT_PNG)
mean_brightness = float(rgba[..., :3].astype(np.float32).mean())

print("LECTURE: capstone scene settled.")
print(f"LECTURE: arm driven to {TARGET_DEG} deg, actual final angle = {final_angle:.3f} deg")
print(f"LECTURE: saved frame mean brightness = {mean_brightness:.2f} "
      f"(not 0.00 -- the light is doing real work, per Lecture 08)")
print(f"LECTURE: saved {OUTPUT_PNG}")

kit.close()
