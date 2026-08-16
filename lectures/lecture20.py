"""Lecture 20 -- Isaac Lab environments 101: the Gym registry and vectorized envs.

Run it (note: a DIFFERENT toolchain than every earlier lecture -- see lecture20.md):
    <your-isaac-lab-install>/isaaclab.sh -p <this-repo>/lectures/lecture20.py

Every lecture through 19 built a `SimulationApp`, authored a `UsdPhysics.Scene`
and every prim in it by hand, and drove physics with an explicit
`for step: ...; kit.update()` loop. That's the layer Isaac Lab's
`ManagerBasedRLEnv` is built ON TOP OF, not a replacement for it: the same
`ArticulationRootAPI`, the same PD-drive joints Lecture 09 hand-tuned, the
same physics scene -- just assembled from a declarative config class instead
of imperative USD calls, registered with `gymnasium` so `gym.make(task_id)`
hands you a ready-to-step, *vectorized* environment (thousands of physics-
identical copies of the same scene, stepped together on one GPU) instead of
the single scene every earlier lecture built.

This lecture doesn't train anything -- it inspects what `gym.make()` actually
handed back, the same "check, don't assume" habit this course has applied to
lidar fields (Lecture 11), depth conventions (Lecture 14), and wheel odometry
(Lecture 18), now applied to an Isaac Lab environment before Lecture 21 trains
a real policy in it.

A quirk specific to this toolchain, worth knowing before you go looking for
your own output: partway through `gym.make()` building the vectorized scene,
Kit re-points file descriptor 1 as part of its own logging setup. Everything
printed *before* that point still reaches a redirected log file; everything
printed after -- including plain `print()` -- silently vanishes from it, even
though the Python process keeps running correctly (confirmed by writing
markers straight to a file at every step: execution reaches the very end,
the prints just never arrive in the redirected stdout). So this script writes
its findings straight to `lecture20_results.txt` next to this file instead of
trusting captured console output.
"""

import argparse
import pathlib

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Lecture 20 -- Isaac Lab environments 101")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import isaaclab_tasks  # noqa: E402,F401 -- importing this module is what registers every Isaac Lab task with gymnasium; nothing renders without it
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

RESULTS_PATH = pathlib.Path(__file__).parent / "lecture20_results.txt"
_lines = []


def report(msg):
    print(msg)
    _lines.append(msg)
    # Written after every line, not just at the end: if Kit's own shutdown
    # path ever aborts the process early, whatever ran so far is still on
    # disk instead of lost with the swallowed stdout.
    RESULTS_PATH.write_text("\n".join(_lines) + "\n")


TASK = "Isaac-Velocity-Flat-G1-v0"
NUM_ENVS = 8
N_STEPS = 100

env_cfg = parse_env_cfg(TASK, device=str(args_cli.device), num_envs=NUM_ENVS)
env = gym.make(TASK, cfg=env_cfg)

report(f"LECTURE: task={TASK}")
report(f"LECTURE: env.unwrapped.num_envs={env.unwrapped.num_envs} (requested {NUM_ENVS})")
report(f"LECTURE: observation_space={env.observation_space}")
report(f"LECTURE: action_space={env.action_space}")

# The action space bounds are the first thing worth checking rather than
# assuming: a 37-dim action vector for a 37-DOF humanoid LOOKS like it could
# be a bounded, normalized range like [-1, 1]. It is not -- Gym reports the
# raw Box as unbounded (-inf, inf); nothing in the space itself clips the
# policy's output. The actual limit comes from elsewhere: the action manager
# multiplies each component by a per-joint `action_scale` and adds it to that
# joint's default position to get the PD target, and the PD drive's own
# joint-limit/effort clamps (Lecture 09) do the rest. Confirm the real
# bounds instead of assuming a convention.
low = env.action_space.low
high = env.action_space.high
report(f"LECTURE: action bounds -- min={float(low.min()):.3f} max={float(high.max()):.3f} "
       f"(unbounded Gym Box, NOT a normalized [-1, 1] range and NOT raw joint angles in "
       f"radians -- the action manager scales this by a per-joint `action_scale` and adds "
       f"it to each joint's default position to get the actual PD target)")

obs, info = env.reset()
policy_obs = obs["policy"]
report(f"LECTURE: reset() obs['policy'] shape={tuple(policy_obs.shape)}, device={policy_obs.device}")

assert policy_obs.shape[0] == NUM_ENVS, "batch dimension should equal num_envs -- this is vectorized, not one scene"
assert str(policy_obs.device).startswith("cuda"), (
    f"expected observations on the requested CUDA device, got {policy_obs.device} -- "
    "device placement is worth checking explicitly, not assuming parse_env_cfg's device= actually took effect"
)

# Zero actions do NOT mean "apply no control" -- they mean "PD target =
# each joint's default position + 0 * action_scale", i.e. hold the robot's
# authored default standing pose. Whether that pose is actually a stable
# equilibrium under this env's randomized velocity *commands* (it samples a
# nonzero target base velocity every episode, same as Lecture 17's "walking"
# command) is exactly the kind of thing to run and check, not assume.
zero_actions = torch.zeros(env.unwrapped.action_space.shape, device=env.unwrapped.device)
total_reward = torch.zeros(NUM_ENVS, device=env.unwrapped.device)
terminated_ever = torch.zeros(NUM_ENVS, dtype=torch.bool, device=env.unwrapped.device)

for _ in range(N_STEPS):
    obs, reward, terminated, truncated, info = env.step(zero_actions)
    total_reward += reward
    terminated_ever |= terminated

n_terminated = int(terminated_ever.sum())
report(f"LECTURE: ran {N_STEPS} vectorized steps across {NUM_ENVS} envs, holding the default pose "
       f"(zero action) the entire time")
report(f"LECTURE: per-env total reward over {N_STEPS} steps: "
       f"{[round(v, 2) for v in total_reward.cpu().tolist()]}")
report(f"LECTURE: envs that terminated at least once (base contact / fall) = {n_terminated}/{NUM_ENVS}")

env.close()
simulation_app.close()
