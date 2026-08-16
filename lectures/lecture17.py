"""Lecture 17 -- Humanoid locomotion with a ready-to-use policy (Unitree H1).

Run it:
    <isaac-sim>/python.sh lectures/lecture17.py

Lecture 09 drove a single revolute joint by hand: a fixed target angle, a
stiffness/damping pair you chose, and a controller loop that just held
that one setpoint under load. A humanoid can't be walked that way --
there's no fixed joint target for "walk forward" across dozens of
coupled joints while staying balanced. `H1FlatTerrainPolicy` turns out to
be the *same* PD-drive machinery from Lecture 09 (force-mode
articulation, stiffness/damping gains, applied every physics step) with
one difference: the target fed into that PD loop each control tick comes
from a neural network NVIDIA trained with RL, not a constant you picked.
This lecture spawns two H1 robots side by side -- one commanded to stand
still, one commanded to walk forward -- and checks that the same
pretrained policy actually produces different, physically sane behavior
for each, without training or tuning anything.

A trap this lecture caught: every earlier lecture drove physics with a
plain `for step: ...; kit.update()` loop, one `kit.update()` == one
physics step. That assumption breaks the moment rendering and physics
are decoupled (`RenderingManager.set_dt()`, used below because the H1
policy expects a stable 200Hz physics rate while rendering headlessly
doesn't need to match it) -- a single `kit.update()` can then advance
physics *multiple* ticks internally before Python regains control. A
loop that calls `policy.forward()` once per `kit.update()` only refreshes
the PD target once per several physics ticks; the articulation free-falls
under a stale target in between, and a bipedal robot standing on two
small contact patches collapses within a few tenths of a second -- while
the printed summary still looks like it "ran". The fix, matching
NVIDIA's own H1 example, is to drive `forward()` from a physics-step
callback (`SimulationManager.register_callback(..., POST_PHYSICS_STEP)`)
so it runs exactly once per physics tick regardless of render pacing.
"""

import os

import numpy as np
from isaacsim import SimulationApp

kit = SimulationApp({"headless": True, "multi_gpu": False, "active_gpu": 0})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.deprecation_manager import import_module  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents  # noqa: E402
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import UsdGeom, UsdLux, UsdPhysics  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

torch = import_module("torch")

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

# A flat ground plane, built the same way every earlier lecture's floor
# was -- a scaled UsdGeom.Cube collider with an explicit friction
# material -- not the streamed environment USD NVIDIA's own H1 example
# references. The robot itself (h1.usd + its trained policy .pt) still
# has to come from Nucleus; there's no authoring a trained locomotion
# policy from primitives. That download turned out fast (a few seconds,
# not the CDN slowness documented for pip/pypi on this network) because
# USD reference resolution and a small TorchScript file are nothing like
# a multi-hundred-MB Python wheel.
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(1.0)
UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -0.05))
UsdGeom.XformCommonAPI(ground).SetScale((20.0, 20.0, 0.1))
ground_prim = ground.GetPrim()
UsdPhysics.CollisionAPI.Apply(ground_prim)
material = UsdPhysics.MaterialAPI.Apply(ground_prim)
material.CreateStaticFrictionAttr(1.0)
material.CreateDynamicFrictionAttr(0.8)
material.CreateRestitutionAttr(0.0)

# A light and a camera, purely so this lecture's .md has a real frame to
# show -- same key + fill dome pattern Lecture 10's capstone used, and the
# same offset-based aim Lectures 02/03 verified: camera placed at
# target + (5,-5,4), rotate=(60,0,45), looks straight at whatever "target"
# is regardless of where that target sits in the world.
key_light = UsdLux.SphereLight.Define(stage, "/World/KeyLight")
key_light.CreateRadiusAttr(0.3)
key_light.CreateIntensityAttr(30000.0)
UsdGeom.XformCommonAPI(key_light).SetTranslate((-1.0, 0.75, 3.5))

fill_dome = UsdLux.DomeLight.Define(stage, "/World/FillDome")
fill_dome.CreateIntensityAttr(400.0)

# Midpoint between where standing stays (near its start) and where walking
# ends up after ~1.3m of forward travel -- and a wider offset than Lectures
# 02/03/10 used, because two robots 1.5m apart need more of the frame than
# one box does at the same relative distance.
CAM_TARGET = (0.65, 0.75, 1.0)
cam_geom = UsdGeom.Camera.Define(stage, "/World/Cam")
UsdGeom.XformCommonAPI(cam_geom).SetTranslate(tuple(t + o for t, o in zip(CAM_TARGET, (9.0, -9.0, 7.0))))
UsdGeom.XformCommonAPI(cam_geom).SetRotate((60.0, 0.0, 45.0))
overview_cam = Camera(prim_path="/World/Cam", resolution=(480, 360))
overview_cam.initialize()

# Physics steps at 200Hz; rendering only needs to keep up every 8th
# physics tick (25Hz) since this runs headless. This is the setting that
# makes a plain per-iteration loop unsafe -- see the module docstring.
RenderingManager.set_dt(8.0 / 200.0)
SimulationManager.set_physics_sim_device("cpu")
SimulationManager.set_physics_dt(1.0 / 200.0)

# Two robots, same USD, same trained policy weights -- only the command
# vector fed to forward() differs. 1.5m apart in Y so they don't collide.
standing = H1FlatTerrainPolicy(prim_path="/World/H1_standing", position=[0.0, 0.0, 1.05])
walking = H1FlatTerrainPolicy(prim_path="/World/H1_walking", position=[0.0, 1.5, 1.05])

decimation = walking.policy_env_params.get("decimation")
policy_dt = walking.policy_env_params.get("sim", {}).get("dt")
print(f"LECTURE: policy trained at decimation={decimation}, sim dt={policy_dt}s -- "
      f"policy inference runs every {decimation} physics steps "
      f"({1.0 / (decimation * policy_dt):.1f} Hz effective control rate), "
      f"physics itself steps at {1.0 / policy_dt:.0f} Hz")

standing_command = torch.zeros(3)
walking_command = torch.tensor([0.5, 0.0, 0.0])  # v_x, v_y, w_z (m/s, m/s, rad/s)

N_STEPS = 600  # 3s of sim time at 200Hz
dt = 1.0 / 200.0

state = {"initialized": False, "step": 0}
pos0_standing = None
pos0_walking = None
min_height_standing = float("inf")
min_height_walking = float("inf")
TRACE_EVERY = 5  # ~120 samples over 600 steps -- enough for a smooth curve, small enough to dump cheaply
trace_t, trace_x_standing, trace_x_walking = [], [], []


def on_physics_step(step_size: float, _context: object) -> None:
    global pos0_standing, pos0_walking, min_height_standing, min_height_walking
    if not state["initialized"]:
        standing.initialize()
        walking.initialize()
        pos0_standing = standing.robot.get_world_poses()[0].numpy()[0].copy()
        pos0_walking = walking.robot.get_world_poses()[0].numpy()[0].copy()
        state["initialized"] = True
        return
    standing.forward(step_size, standing_command)
    walking.forward(step_size, walking_command)
    state["step"] += 1
    pos_standing = standing.robot.get_world_poses()[0].numpy()[0]
    pos_walking = walking.robot.get_world_poses()[0].numpy()[0]
    h_standing = float(pos_standing[2])
    h_walking = float(pos_walking[2])
    min_height_standing = min(min_height_standing, h_standing)
    min_height_walking = min(min_height_walking, h_walking)
    if state["step"] % TRACE_EVERY == 0:
        trace_t.append(state["step"] * step_size)
        trace_x_standing.append(float(pos_standing[0]) - float(pos0_standing[0]))
        trace_x_walking.append(float(pos_walking[0]) - float(pos0_walking[0]))


SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
kit.update()  # first physics tick: fires the callback's initialize() branch

while state["step"] < N_STEPS:
    kit.update()

pos1_standing = standing.robot.get_world_poses()[0].numpy()[0].copy()
pos1_walking = walking.robot.get_world_poses()[0].numpy()[0].copy()

elapsed_s = N_STEPS * dt
disp_standing = float(np.linalg.norm(pos1_standing[:2] - pos0_standing[:2]))
disp_walking_forward = float(pos1_walking[0] - pos0_walking[0])  # +X is forward for this command
disp_walking_total = float(np.linalg.norm(pos1_walking[:2] - pos0_walking[:2]))
naive_implied = float(walking_command[0]) * elapsed_s

print(f"LECTURE: ran {state['step']} physics steps ({elapsed_s:.1f}s sim time)")
print(f"LECTURE: standing (command=[0,0,0])   -> XY drift={disp_standing:.3f} m, "
      f"min base height={min_height_standing:.3f} m")
print(f"LECTURE: walking  (command=[0.5,0,0]) -> +X advance={disp_walking_forward:.3f} m, "
      f"total XY={disp_walking_total:.3f} m, min base height={min_height_walking:.3f} m")
print(f"LECTURE: naive implied distance at 0.5 m/s x {elapsed_s:.1f}s = {naive_implied:.3f} m "
      f"-- compare to the {disp_walking_forward:.3f} m actually measured (ramp-up + real tracking error, not equal)")

assert min_height_standing > 0.6, "standing robot fell over"
assert min_height_walking > 0.6, "walking robot fell over"
assert disp_walking_forward > disp_standing * 3, "walking robot didn't move meaningfully more than the standing one"
print("LECTURE: both robots stayed upright, and the walking command produced clearly more "
      "forward motion than standing still -- verified against real physics output, not assumed")

# A real captured frame of the final pose, and the position traces recorded
# during the run above -- so this lecture's .md can show what "stayed
# upright" and "walked forward" actually looked like, not just assert it.
rgba = None
for _ in range(60):
    kit.update()
    candidate = overview_cam.get_rgba()
    if candidate is not None and candidate.size > 0:
        rgba = candidate
if rgba is None:
    raise RuntimeError("overview camera never produced a frame")
overview_path = os.path.join(HERE, "output_lecture17_overview.png")
Image.fromarray(rgba, mode="RGBA").save(overview_path)
print(f"LECTURE: saved overview frame to {overview_path}")

np.savez(
    os.path.join(HERE, "data_lecture17.npz"),
    t=np.array(trace_t),
    x_standing=np.array(trace_x_standing),
    x_walking=np.array(trace_x_walking),
)

kit.close()
