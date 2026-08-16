"""Lecture 21 -- Training G1 to walk with PPO, and checking what training bought you.

Run it (same toolchain as Lecture 20 -- see lecture21.md for the full story,
including the actual training command used):
    <your-isaac-lab-install>/isaaclab.sh -p <this-repo>/lectures/lecture21.py \
        --checkpoint <path-to-your-model_N.pt>

This lecture does NOT run a fresh 500-iteration training job itself -- that
took Isaac Lab's own `scripts/reinforcement_learning/rsl_rl/train.py`
roughly 5 minutes on this machine (num_envs=2048, --headless --device
cuda:0) and produced real checkpoints under
`<isaaclab-install>/logs/rsl_rl/g1_flat/<timestamp>/model_N.pt`. Re-deriving
RSL-RL's PPO loop by hand here would just be a worse copy of a script Isaac
Lab already ships and tests -- see lecture21.md for the exact command and
the full reward curve.

What this script DOES do: load one of that run's own checkpoints and rerun
*the same* task/NUM_ENVS/N_STEPS window Lecture 20 already measured with
zero actions (8/8 environments terminated within 100 steps holding the
default pose) -- this time driving the loaded, trained policy's own
actions instead. It intentionally does NOT also recreate the untrained
pass here: a second `gym.make()` call for a second full scene, in the same
process, after the first one's `env.close()`, was tried and hangs
indefinitely (worth knowing on its own -- see lecture21.md). Two full Kit
environments in one process is a real, separate gotcha from Lecture 20's
fd-1 one; the practical fix is what every earlier lecture already did:
one environment per process, one `isaaclab.sh -p` invocation per number
you need.
"""

import argparse
import pathlib

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Lecture 21 -- G1 PPO: trained-policy rollout")
parser.add_argument("--checkpoint", type=str, required=True,
                     help="Path to a rsl_rl model_N.pt checkpoint, e.g. "
                          "<isaaclab-install>/logs/rsl_rl/g1_flat/<timestamp>/model_499.pt")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import isaaclab_tasks  # noqa: E402,F401 -- registers every Isaac Lab task with gymnasium
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    G1FlatPPORunnerCfg,
)
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

RESULTS_PATH = pathlib.Path(__file__).parent / "lecture21_results.txt"
_lines = []


def report(msg):
    print(msg)
    _lines.append(msg)
    # Same reason as Lecture 20: Kit's fd-1 repoint mid-`gym.make()` swallows
    # console output after this point, so results go straight to a file.
    RESULTS_PATH.write_text("\n".join(_lines) + "\n")


TASK = "Isaac-Velocity-Flat-G1-v0"
NUM_ENVS = 8
N_STEPS = 100
UNTRAINED_TERMINATED = 8  # Lecture 20's already-verified zero-action result, out of NUM_ENVS=8

env_cfg = parse_env_cfg(TASK, device=str(args_cli.device), num_envs=NUM_ENVS)
env = gym.make(TASK, cfg=env_cfg)
env = RslRlVecEnvWrapper(env, clip_actions=None)

agent_cfg = G1FlatPPORunnerCfg()
ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=str(args_cli.device))
ppo_runner.load(args_cli.checkpoint)
report(f"LECTURE: loaded checkpoint {args_cli.checkpoint}")
policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

obs, _ = env.get_observations()
total_reward = torch.zeros(NUM_ENVS, device=env.unwrapped.device)
terminated_ever = torch.zeros(NUM_ENVS, dtype=torch.bool, device=env.unwrapped.device)
with torch.inference_mode():
    for _ in range(N_STEPS):
        actions = policy(obs)
        obs, reward, done, extra = env.step(actions)
        total_reward += reward
        terminated_ever |= done.bool()
n_terminated = int(terminated_ever.sum())

report(f"LECTURE: [trained policy] total reward per env over {N_STEPS} steps: "
       f"{[round(v, 2) for v in total_reward.cpu().tolist()]}")
report(f"LECTURE: [trained policy] envs that terminated at least once = {n_terminated}/{NUM_ENVS}")
report(f"LECTURE: comparison -- Lecture 20's untrained zero-action pass terminated "
       f"{UNTRAINED_TERMINATED}/{NUM_ENVS} over the same {N_STEPS}-step/{NUM_ENVS}-env window; "
       f"this trained checkpoint terminated {n_terminated}/{NUM_ENVS}")

env.close()
simulation_app.close()
