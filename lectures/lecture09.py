"""Lecture 09 -- Articulations and Joint Drives.

Run it:
    <isaac-sim>/python.sh lectures/lecture09.py
"""

import math

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

TARGET_DEG = 45.0
CHECKPOINTS = (0, 5, 10, 15, 20, 30, 50, 80, 120)
BASE_POS = (0.0, 0.0, 1.0)


def build_single_joint_arm(stiffness, damping):
    """One fixed base, one arm link, one revolute joint between them, one
    position drive on that joint. As small as an articulation gets -- a
    real robot arm is this exact pattern repeated per joint.
    """
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    # The base: a small static-feeling body. RigidBodyAPI is still applied
    # (a FixedJoint below is what actually pins it, not the absence of
    # RigidBodyAPI) and ArticulationRootAPI marks it as the root of a
    # reduced-coordinate PhysX articulation -- the thing that makes a
    # chain of joints get solved as one connected mechanism instead of a
    # pile of separately-constrained rigid bodies.
    base = UsdGeom.Cube.Define(stage, "/World/Base")
    base.CreateSizeAttr(0.2)
    UsdGeom.XformCommonAPI(base).SetTranslate(BASE_POS)
    UsdPhysics.CollisionAPI.Apply(base.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(base.GetPrim())

    # Pin the base to the world itself -- a joint with only Body1 set, so
    # side 0 of the joint IS the world frame. That means side 0's local
    # anchor has to be given in WORLD coordinates to land on the same
    # physical point Base actually occupies -- leave it at the (0, 0, 0)
    # default and PhysX anchors side 0 to the world ORIGIN instead, and
    # since side 0 can't move (it's the world), it's Base -- the side that
    # CAN move -- that gets silently dragged there the instant physics
    # starts. This is exactly that mistake, caught by checking it rather
    # than assuming a fixed joint automatically fixes something in place
    # wherever you put it.
    fixed = UsdPhysics.FixedJoint.Define(stage, "/World/Base/FixedToWorld")
    fixed.CreateBody1Rel().SetTargets(["/World/Base"])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*BASE_POS))

    # The arm: an Xform whose ORIGIN sits exactly at the base's position --
    # that's the pivot point -- with its visible geometry offset half a
    # meter away from that origin. Separating "where the arm rotates about"
    # from "where the arm's box is drawn" is what makes the joint's local
    # anchor points both (0, 0, 0) below, instead of some offset that has
    # to be computed by hand.
    arm = UsdGeom.Xform.Define(stage, "/World/Arm")
    UsdGeom.XformCommonAPI(arm).SetTranslate(BASE_POS)
    UsdPhysics.RigidBodyAPI.Apply(arm.GetPrim())
    UsdPhysics.MassAPI.Apply(arm.GetPrim()).CreateMassAttr(0.5)

    arm_geom = UsdGeom.Cube.Define(stage, "/World/Arm/Geom")
    arm_geom.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(arm_geom).SetScale((1.0, 0.1, 0.1))
    UsdGeom.XformCommonAPI(arm_geom).SetTranslate((0.5, 0.0, 0.0))
    UsdPhysics.CollisionAPI.Apply(arm_geom.GetPrim())

    # The joint: hinges /World/Arm relative to /World/Base about the Y
    # axis, anchored at (0, 0, 0) in both bodies' own local frames --
    # which, per the paragraph above, is the same physical point in the
    # world for both of them.
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Base/Hinge")
    joint.CreateBody0Rel().SetTargets(["/World/Base"])
    joint.CreateBody1Rel().SetTargets(["/World/Arm"])
    joint.CreateAxisAttr("Y")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    # The drive: not a teleport to the target angle, a spring-damper
    # regulator pulling toward it -- stiffness resists being away from
    # the target, damping resists moving quickly, same shape as a PD
    # controller because that's functionally what this is.
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(TARGET_DEG)
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(1e6)

    return arm


def arm_angle_deg(arm):
    """The joint doesn't expose "current angle" as a single number to read
    -- it's derived here from the arm's own world transform, same
    technique Lecture 03 used: transform a known local direction (+X, the
    direction the arm's geometry extends in) into world space and measure
    the angle it landed at.
    """
    m = UsdGeom.Xformable(arm.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    forward = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return math.degrees(math.atan2(-forward[2], forward[0]))


def run_and_trace(stiffness, damping):
    arm = build_single_joint_arm(stiffness, damping)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    trace = {}
    for step in range(max(CHECKPOINTS) + 1):
        if step in CHECKPOINTS:
            trace[step] = arm_angle_deg(arm)
        kit.update()
    timeline.stop()
    for _ in range(3):
        kit.update()
    return trace


light_damping = run_and_trace(stiffness=150.0, damping=5.0)
heavy_damping = run_and_trace(stiffness=150.0, damping=25.0)

print(f"LECTURE: driving a single hinge joint to a {TARGET_DEG} deg target, against gravity")
print(f"LECTURE: {'step':>5} {'damping=5 (deg)':>18} {'damping=25 (deg)':>18}")
for step in sorted(light_damping):
    print(f"LECTURE: {step:>5} {light_damping[step]:>18.3f} {heavy_damping[step]:>18.3f}")

print(f"\nLECTURE: damping=5 settles at {light_damping[max(CHECKPOINTS)]:.3f} deg "
      f"(reaches the target fast)")
print(f"LECTURE: damping=25 settles at {heavy_damping[max(CHECKPOINTS)]:.3f} deg "
      f"(reaches it smoothly, slower, and neither value is exactly "
      f"{TARGET_DEG} -- see lecture09.md)")

import os  # noqa: E402

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
steps = np.array(sorted(light_damping))
np.savez(
    os.path.join(HERE, "data_lecture09.npz"),
    steps=steps,
    light_damping=np.array([light_damping[s] for s in steps]),
    heavy_damping=np.array([heavy_damping[s] for s in steps]),
    target_deg=TARGET_DEG,
)

kit.close()
