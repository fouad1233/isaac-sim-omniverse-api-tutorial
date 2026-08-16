"""Lecture 05 -- Physics Scene & Rigid Bodies.

Run it:
    <isaac-sim>/python.sh lectures/lecture05.py
"""

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402


def build_falling_box_scene():
    """A static ground (collision, no rigid body -- it never moves) and a
    cube 2 m above it with collision + rigid body + mass, so it's eligible
    to be simulated. Returns the box prim so the caller can watch it fall.
    """
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # A static collider: CollisionAPI without RigidBodyAPI. PhysX treats
    # this as immovable geometry -- things can hit it, it never hits back.
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(10.0)
    UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -5.0))  # top face at z=0
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # A dynamic body: CollisionAPI (so it can hit things) + RigidBodyAPI (so
    # PhysX picks it up and simulates it at all) + MassAPI (an explicit mass
    # -- skip this and PhysX estimates one from the collision geometry and a
    # default density instead, which is fine but not something you chose).
    box = UsdGeom.Cube.Define(stage, "/World/FallingBox")
    box.CreateSizeAttr(0.2)
    UsdGeom.XformCommonAPI(box).SetTranslate((0.0, 0.0, 2.0))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr(1.0)

    return stage, box


def height_of(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return m.ExtractTranslation()[2]


def run_and_trace(gravity_magnitude, checkpoints=(0, 10, 20, 30, 40, 60, 80, 100)):
    """Build a fresh scene, optionally author a physics scene with the given
    gravity magnitude (None = author none at all), step through it, and
    return {step: height} at the requested checkpoints.
    """
    stage, box = build_falling_box_scene()
    if gravity_magnitude is not None:
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(gravity_magnitude)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    trace = {}
    for step in range(max(checkpoints) + 1):
        if step in checkpoints:
            trace[step] = height_of(box)
        kit.update()
    timeline.stop()
    for _ in range(3):
        kit.update()
    return trace


# Older Isaac Sim versions required an explicit UsdPhysics.Scene or nothing
# simulated at all -- a rigid body just hangs in the air forever, frozen,
# even with collision and mass correctly authored. On THIS Isaac Sim
# version, that turns out not to be true any more (verified below, not
# assumed) -- omni.physx now falls back to an implicit default scene when
# none is authored. Don't take either claim on faith for whatever version
# you're running; this is exactly the kind of thing that changes silently
# between releases, which is the actual point of running all three cases
# and comparing numbers instead of reading about it.
none_trace = run_and_trace(None)
earth_trace = run_and_trace(9.81)
weak_trace = run_and_trace(2.0)

print("LECTURE: height (m) over time -- three scenes, same starting height 2.0 m")
print(f"LECTURE: {'step':>5} {'no scene (implicit)':>20} {'scene, g=9.81':>15} {'scene, g=2.0':>15}")
for step in sorted(none_trace):
    print(
        f"LECTURE: {step:>5} {none_trace[step]:>20.4f} "
        f"{earth_trace[step]:>15.4f} {weak_trace[step]:>15.4f}"
    )

matches_earth = all(
    abs(none_trace[s] - earth_trace[s]) < 1e-6 for s in none_trace
)
print(f"\nLECTURE: implicit (no scene) trace exactly matches explicit g=9.81 trace: {matches_earth}")
print("LECTURE: (that's not a coincidence -- see lecture05.md)")

import os  # noqa: E402

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
steps = np.array(sorted(none_trace))
np.savez(
    os.path.join(HERE, "data_lecture05.npz"),
    steps=steps,
    none_heights=np.array([none_trace[s] for s in steps]),
    earth_heights=np.array([earth_trace[s] for s in steps]),
    weak_heights=np.array([weak_trace[s] for s in steps]),
)

kit.close()
