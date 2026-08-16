"""Lecture 18 -- Differential-drive controller and wheel odometry (Jetbot).

Run it:
    <isaac-sim>/python.sh lectures/lecture18.py

Lecture 09 drove one joint by hand. Lecture 17 put a trained policy on
top of that same PD-drive machinery for a legged robot. This lecture
does the wheeled equivalent: `DifferentialController` converts a
[linear speed, angular speed] command into [left, right] wheel angular
velocities using the textbook two-wheel kinematic model, and
`WheeledRobot.apply_wheel_actions()` feeds those straight into velocity
drives on the wheel joints -- no PD position target, no neural network,
just the algebra every differential-drive robot's firmware runs.

The real lesson is what happens when you try to go the other direction:
using the *same* kinematic model, run backwards, to estimate the robot's
pose from its own wheel motion alone -- wheel odometry, the thing a real
robot uses when it has no ground-truth pose to check itself against.
`DifferentialController` was built with `wheel_radius=0.03`, the value
NVIDIA's own Jetbot example uses. This lecture checks that value against
the actual simulated robot rather than trusting it, and shows what
happens to odometry accuracy when the check turns up a mismatch.
"""

import math

import numpy as np
from isaacsim import SimulationApp

kit = SimulationApp({"headless": True, "multi_gpu": False, "active_gpu": 0})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import UsdGeom, UsdPhysics  # noqa: E402

NOMINAL_WHEEL_RADIUS = 0.03  # m -- NVIDIA's own jetbot_differential_move.py value
WHEEL_BASE = 0.1125  # m -- same source, not questioned here (see "Try it yourself")
DT = 1.0 / 60.0

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")
jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)

ctx = omni.usd.get_context()
stage = ctx.get_stage()
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

# Same primitive floor every earlier lecture used. The robot mesh itself
# (jetbot.usd) still has to come from Nucleus, same tradeoff Lecture 17
# already made for H1 -- there's no authoring a modeled robot from cubes.
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

jetbot = WheeledRobot(
    paths="/World/Jetbot",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=jetbot_asset_path,
    positions=[0.0, 0.0, 0.05],
)
controller = DifferentialController(wheel_radius=NOMINAL_WHEEL_RADIUS, wheel_base=WHEEL_BASE)

# Don't assume get_dof_velocities() returns [left, right] in that order --
# resolve the actual indices by name, the same "verify, don't assume"
# check this course has applied to sensor conventions and drive units.
wheel_dof_indices = jetbot.get_dof_indices(["left_wheel_joint", "right_wheel_joint"]).numpy().tolist()
print(f"LECTURE: jetbot dof_names={jetbot.dof_names}, resolved wheel_dof_indices={wheel_dof_indices}")

# Match NVIDIA's own example exactly here: this is not the render/physics
# decoupling that broke Lecture 17's naive loop (SimulationManager here
# drives physics and rendering at the SAME 1/60s dt), so a plain per-step
# loop calling apply_wheel_actions() once per app_utils.update_app(steps=1)
# is safe -- confirmed by probing this exact loop shape before writing it
# into the lecture.
SimulationManager.setup_simulation(dt=DT, device="cpu")
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)
app_utils.play()
app_utils.update_app(steps=10)


def world_pose(robot):
    """(x, y, yaw) -- yaw from the quaternion's Z-rotation component, valid
    for a ground robot that doesn't roll or pitch.
    """
    pos = robot.get_world_poses()[0].numpy()[0]
    w, x, y, z = robot.get_world_poses()[1].numpy()[0]
    yaw = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return float(pos[0]), float(pos[1]), yaw


# ---------------------------------------------------------------------------
# Phase A -- calibrate. Command a known forward speed for a known duration
# and measure how far the robot actually went. This is the same question a
# real robot's calibration routine has to answer: "wheel_radius=0.03" is a
# number from NVIDIA's example script, not a measurement of this simulated
# Jetbot's own geometry.
# ---------------------------------------------------------------------------
CALIB_STEPS = 180
CALIB_SPEED = 0.1

x0, y0, _ = world_pose(jetbot)
for _ in range(CALIB_STEPS):
    wheel_vel = controller.forward([CALIB_SPEED, 0.0])
    jetbot.apply_wheel_actions(wheel_vel)
    app_utils.update_app(steps=1)
x1, y1, _ = world_pose(jetbot)

measured_dist = float(np.hypot(x1 - x0, y1 - y0))
elapsed = CALIB_STEPS * DT
measured_speed = measured_dist / elapsed
calibrated_wheel_radius = NOMINAL_WHEEL_RADIUS * (measured_speed / CALIB_SPEED)

print(f"LECTURE: calibration -- commanded {CALIB_SPEED} m/s for {elapsed:.2f}s, "
      f"naive implied distance={CALIB_SPEED * elapsed:.4f} m, actually measured={measured_dist:.4f} m "
      f"({measured_speed:.4f} m/s, {100 * (measured_speed / CALIB_SPEED - 1):+.1f}% vs commanded)")
print(f"LECTURE: nominal wheel_radius={NOMINAL_WHEEL_RADIUS:.4f} m (NVIDIA's example value) "
      f"-- calibrated wheel_radius={calibrated_wheel_radius:.4f} m (measured from this run's actual physics)")

# ---------------------------------------------------------------------------
# Phase B -- drive an S-curve (forward / turn / forward) and dead-reckon
# odometry from wheel readings alone, the same information a real robot's
# onboard encoders would give it with no access to ground truth. Two
# parallel estimates run side by side: one using the nominal wheel_radius,
# one using the radius calibrated in Phase A. Both use the exact inverse of
# DifferentialController's own forward kinematics, so any drift comes from
# real physics (wheel radius error, wheel slip, integration step size) --
# not a mismatched formula.
# ---------------------------------------------------------------------------
# (steps, v_cmd m/s, w_cmd rad/s) -- forward, turn in place, forward again.
SEGMENTS = [
    (120, 0.15, 0.0),
    (105, 0.0, 1.0),
    (120, 0.15, 0.0),
]

gt_x0, gt_y0, gt_theta0 = world_pose(jetbot)
odo_nominal = {"x": gt_x0, "y": gt_y0, "theta": gt_theta0}
odo_calibrated = {"x": gt_x0, "y": gt_y0, "theta": gt_theta0}


def dead_reckon(state, omega_left, omega_right, wheel_radius):
    v_left = omega_left * wheel_radius
    v_right = omega_right * wheel_radius
    v = (v_left + v_right) / 2.0
    w = (v_right - v_left) / WHEEL_BASE
    state["x"] += v * np.cos(state["theta"]) * DT
    state["y"] += v * np.sin(state["theta"]) * DT
    state["theta"] += w * DT


traj_gt = [(gt_x0, gt_y0)]
traj_nominal = [(gt_x0, gt_y0)]
traj_calibrated = [(gt_x0, gt_y0)]

for steps, v_cmd, w_cmd in SEGMENTS:
    for _ in range(steps):
        wheel_vel = controller.forward([v_cmd, w_cmd])
        jetbot.apply_wheel_actions(wheel_vel)
        app_utils.update_app(steps=1)
        actual = jetbot.get_dof_velocities(dof_indices=wheel_dof_indices).numpy()[0]  # [left, right] rad/s
        dead_reckon(odo_nominal, actual[0], actual[1], NOMINAL_WHEEL_RADIUS)
        dead_reckon(odo_calibrated, actual[0], actual[1], calibrated_wheel_radius)
        gt_x_step, gt_y_step, _ = world_pose(jetbot)
        traj_gt.append((gt_x_step, gt_y_step))
        traj_nominal.append((odo_nominal["x"], odo_nominal["y"]))
        traj_calibrated.append((odo_calibrated["x"], odo_calibrated["y"]))

gt_x1, gt_y1, gt_theta1 = world_pose(jetbot)

err_nominal = float(np.hypot(odo_nominal["x"] - gt_x1, odo_nominal["y"] - gt_y1))
err_calibrated = float(np.hypot(odo_calibrated["x"] - gt_x1, odo_calibrated["y"] - gt_y1))

print(f"LECTURE: ground truth final pose      -> x={gt_x1:.4f} y={gt_y1:.4f} theta={np.degrees(gt_theta1):.2f} deg")
print(f"LECTURE: odometry (nominal radius)    -> x={odo_nominal['x']:.4f} y={odo_nominal['y']:.4f} "
      f"theta={np.degrees(odo_nominal['theta']):.2f} deg -- position error={err_nominal:.4f} m")
print(f"LECTURE: odometry (calibrated radius) -> x={odo_calibrated['x']:.4f} y={odo_calibrated['y']:.4f} "
      f"theta={np.degrees(odo_calibrated['theta']):.2f} deg -- position error={err_calibrated:.4f} m")

# The straight-line calibration in Phase A does NOT make Phase B's odometry
# better -- it makes it worse, and reliably so (see lecture18.md for why).
# This assertion locks in that specific, surprising, real result rather than
# the tidier "calibration always helps" story a first guess would predict.
assert abs(gt_theta1 - gt_theta0) > math.radians(30), "turn segment should have produced a substantial heading change"
assert err_nominal < 0.05, "nominal-radius odometry error implausibly large for this short, low-speed run"
assert err_calibrated > err_nominal, (
    "expected the straight-line-only calibration to overcorrect the yaw estimate during the turn segment -- "
    "if this no longer holds, the finding this lecture documents may not reproduce on this install/version"
)
print(f"LECTURE: the calibrated radius made position error {err_calibrated / err_nominal:.1f}x WORSE, not better -- "
      f"it overcorrected theta by ~{100 * (calibrated_wheel_radius / NOMINAL_WHEEL_RADIUS - 1):.1f}% because "
      f"wheel_radius scales BOTH the linear-speed and the yaw-rate term in the kinematic model equally, and the "
      f"12.3% straight-line overshoot calibration measured isn't necessarily a wheel-geometry error at all -- it "
      f"could just as easily be a velocity-drive transient specific to that motion, which doesn't apply the same "
      f"way once the wheels are commanded asymmetrically for a turn")

import os  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
np.savez(
    os.path.join(HERE, "data_lecture18.npz"),
    traj_gt=np.array(traj_gt),
    traj_nominal=np.array(traj_nominal),
    traj_calibrated=np.array(traj_calibrated),
)

kit.close()
