"""Lecture 06 -- The Simulation Loop and Timeline.

Run it:
    <isaac-sim>/python.sh lectures/lecture06.py
"""

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402


def build_falling_box_scene():
    """Same scene as lecture05: a static ground and a box with collision +
    rigid body + mass, plus an explicit physics scene at Earth gravity, so
    the only thing we're studying here is the timeline, not physics setup.
    """
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(10.0)
    UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -5.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    box = UsdGeom.Cube.Define(stage, "/World/FallingBox")
    box.CreateSizeAttr(0.2)
    UsdGeom.XformCommonAPI(box).SetTranslate((0.0, 0.0, 2.0))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr(1.0)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    return box


def height_of(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return m.ExtractTranslation()[2]


# --- Part 1: play / pause / stop are not a symmetric triple -----------------
#
# play() and pause() feel like a toggle. stop() feels like "pause, but more
# so." All three actually do different things to the simulation state, and
# the only way to know which is to watch a physics quantity across each
# transition instead of guessing from the method names.

box = build_falling_box_scene()
timeline = omni.timeline.get_timeline_interface()

# Sampled every update() call, not just at each phase's end -- this is what
# lets the .md show a real curve (falling, then flat, then falling again,
# then a hard reset) instead of four isolated numbers.
phase_heights = []
phase_labels = []

timeline.play()
for _ in range(30):
    kit.update()
    phase_heights.append(height_of(box))
    phase_labels.append("playing")
h_playing = height_of(box)
print(f"LECTURE: playing, 30 steps  -> height={h_playing:.4f}")

timeline.pause()
for _ in range(10):
    kit.update()
    phase_heights.append(height_of(box))
    phase_labels.append("paused")
h_paused = height_of(box)
print(f"LECTURE: paused,  10 more updates -> height={h_paused:.4f}  "
      f"(moved while paused: {abs(h_paused - h_playing) > 1e-6})")

timeline.play()
for _ in range(10):
    kit.update()
    phase_heights.append(height_of(box))
    phase_labels.append("resumed")
h_resumed = height_of(box)
print(f"LECTURE: resumed, 10 more updates -> height={h_resumed:.4f}  "
      f"(continued falling from the paused height, not from the start: "
      f"{h_resumed < h_paused - 1e-4})")

timeline.stop()
for _ in range(5):
    kit.update()
    phase_heights.append(height_of(box))
    phase_labels.append("stopped")
h_stopped = height_of(box)
print(f"LECTURE: stopped, 5 more updates  -> height={h_stopped:.4f}  "
      f"(back to the original authored 2.0: {abs(h_stopped - 2.0) < 1e-3})")


# --- Part 2: kit.update() is a bundle, not an atom --------------------------
#
# Every step so far called kit.update(), which pumps the whole application:
# UI, rendering, and physics together, in that one call. That's convenient,
# but it hides something worth knowing before the next lecture: physics has
# its own interface, and you can step it directly, without touching
# rendering or the timeline at all.

timeline.play()
kit.update()  # let the simulation view initialize after the fresh play()
h_before_direct = height_of(box)

physx_iface = omni.physx.get_physx_interface()
physx_iface.update_simulation(1.0 / 60.0, 0.0)
physx_iface.update_transformations(True, True)
h_after_direct = height_of(box)

print(f"\nLECTURE: height before a direct physx_iface.update_simulation() "
      f"call = {h_before_direct:.4f}")
print(f"LECTURE: height after that direct call (kit.update() never ran) "
      f"= {h_after_direct:.4f}  (physics advanced on its own: "
      f"{abs(h_after_direct - h_before_direct) > 1e-6})")

import os  # noqa: E402

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
np.savez(
    os.path.join(HERE, "data_lecture06.npz"),
    heights=np.array(phase_heights),
    labels=np.array(phase_labels),
)

kit.close()
