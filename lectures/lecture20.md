# Lecture 20 — Isaac Lab environments 101: the Gym registry and vectorized envs

**Code:** [`lecture20.py`](lecture20.py)

> **A different toolchain from here on.** Lectures 1–19 ran directly against
> an Isaac Sim install's `python.sh`. Isaac Lab is a separate install with
> its own launcher and its own pinned Python — see "Run it" below before you
> try to run this the way you ran everything else in this course.

## The one thing this lecture teaches

Every lecture through 19 built a `SimulationApp`, authored a
`UsdPhysics.Scene` and every prim in it by hand, and drove physics with an
explicit `for step: ...; kit.update()` loop. That's the layer Isaac Lab's
`ManagerBasedRLEnv` is built ON TOP OF, not a replacement for it: the same
`ArticulationRootAPI`, the same PD-drive joints Lecture 09 hand-tuned, the
same physics scene — just assembled from a declarative config class instead
of imperative USD calls, and registered with `gymnasium` so `gym.make(task_id)`
hands you a ready-to-step, *vectorized* environment (many physics-identical
copies of the same scene, stepped together on one GPU) instead of the single
scene every earlier lecture built.

This lecture doesn't train anything — it inspects what `gym.make()` actually
handed back, the same "check, don't assume" habit this course has applied to
lidar fields (Lecture 11), depth conventions (Lecture 14), and wheel odometry
(Lecture 18), now applied to an Isaac Lab environment before Lecture 21
trains a real policy in it.

## Run it

```bash
<your-isaac-lab-install>/isaaclab.sh -p <this-repo>/lectures/lecture20.py --headless --device cuda:0
```

This needs an Isaac Lab 2.x install (this course used 2.2.1) with its
bundled Isaac Sim 5.0.0 / Python 3.11 — a separate install from the Isaac
Sim 6.0.1 / Python 3.12 environment every earlier lecture used. The two
Python environments are pinned to different Isaac Sim major versions and
should never be merged. `--device cuda:0`/`cuda:1` picks the GPU the same
way every earlier lecture's `--/physics/cudaDevice=` did — never
`CUDA_VISIBLE_DEVICES` for a full Kit process.

## What you'll see

`lecture20.py` writes its findings to `lecture20_results.txt` next to
itself, **not** just to the console — see "Walking through it" for why that
distinction matters here specifically. Its contents after a real run:

```
LECTURE: task=Isaac-Velocity-Flat-G1-v0
LECTURE: env.unwrapped.num_envs=8 (requested 8)
LECTURE: observation_space=Dict('policy': Box(-inf, inf, (8, 123), float32))
LECTURE: action_space=Box(-inf, inf, (8, 37), float32)
LECTURE: action bounds -- min=-inf max=inf (unbounded Gym Box, NOT a normalized [-1, 1] range and NOT raw joint angles in radians -- the action manager scales this by a per-joint `action_scale` and adds it to each joint's default position to get the actual PD target)
LECTURE: reset() obs['policy'] shape=(8, 123), device=cuda:0
LECTURE: ran 100 vectorized steps across 8 envs, holding the default pose (zero action) the entire time
LECTURE: per-env total reward over 100 steps: [-3.87, -3.82, -2.74, -3.37, -3.81, -3.23, -2.9, -3.28]
LECTURE: envs that terminated at least once (base contact / fall) = 8/8
```

## Walking through it

**123 observations, 37 actions, for a robot Lecture 17 never even needed a
number for.** Unitree G1 is a 37-DOF humanoid (legs, torso, arms, and
fingers), and both the action space's last dimension and Lecture 17's
`H1FlatTerrainPolicy` policy shape (its final `Linear` layer outputs exactly
this many values) agree on that count. The 123-dim policy observation isn't
just joint state — `G1FlatEnvCfg`'s observation manager concatenates base
angular velocity, projected gravity, the velocity *command* the env sampled
for this episode, joint positions and velocities relative to default, the
previous action, and height-scan/other terms depending on config, all as one
flat vector per env. None of that is discoverable by reading the class name;
it's discoverable by checking `observation_space` at runtime, which is
exactly what this script did instead of assuming.

**The action space bounds are `(-inf, inf)`, not `[-1, 1]`.** It would be
reasonable to expect a "normalized" RL action space to be clipped to some
canonical range — that assumption is wrong here, and the printed line above
exists specifically to catch it before Lecture 21 writes any training code
around a false premise. Nothing in the `gym.Box` itself bounds the policy's
output; the actual limits come from downstream — the action manager scales
each component by a per-joint `action_scale` and adds it to that joint's
default position to get a PD target, and the PD drive's own joint-limit and
effort clamps (the same mechanism Lecture 09 hand-tuned) do the rest. A
policy is still free to *propose* an enormous number; it just won't move the
joint past its physical limit.

**Holding the default pose for 100 steps: 8/8 environments fell.** `zero_actions`
does not mean "apply no control" — it means "PD target = default position +
0 × action_scale," i.e. hold the robot's authored standing pose exactly.
Every one of the 8 envs still triggered `base_contact` termination within
100 steps (0.5s of sim time at this env's control rate). This isn't a bug;
it's the actual reason Lecture 21 exists. `G1FlatEnvCfg` samples a nonzero
target base velocity every episode (same idea as Lecture 17's walking
command), and standing rigidly still does not track a nonzero velocity
command — the tracking-error terms in the reward function penalize exactly
that mismatch, and eventually the untracked drift topples the robot. A
trained policy's job is to turn that command into joint motion that doesn't
fall over; an untrained one (or, here, no policy at all) can't.

**A file descriptor gotcha specific to this toolchain, found the hard way.**
The first version of this script only used `print()`. It ran, exited `0`,
and produced zero visible output — not an exception, not a hang, just
silence, which is a more dangerous failure mode than a crash because it
looks like nothing happened rather than looking like something went wrong.
Isolating it took writing a second probe script that stamped a marker to a
plain file after every major line (`AppLauncher` init, each import,
`parse_env_cfg`, `gym.make()`, `env.reset()`, each `print()`, `env.close()`).
That probe proved the Python process really did execute every line,
including the `print()` calls themselves — the marker written immediately
*after* each print was reached every time. What never arrived was the
console output. Isaac Lab's Kit backend re-points file descriptor 1 as part
of its own logging setup partway through `gym.make()`'s scene build;
everything written to stdout before that repoint reaches a redirected log
file, everything after — including plain `print()` — silently disappears
from it, even though the code producing it keeps running correctly. The
fix, and the reason this script writes to `lecture20_results.txt` instead of
trusting captured console output: for any Isaac Lab script whose meaningful
work happens after `gym.make()`, write results straight to a file.

## Try it yourself

1. Change `NUM_ENVS` from `8` to `256` and rerun. Do the per-env total
   rewards over 100 steps of zero action cluster in roughly the same range
   as the 8-env run, or does a much larger batch surface some envs that
   behave very differently (e.g. survive the full 100 steps without
   terminating)? This is the kind of check worth doing before trusting a
   small smoke test to represent full-scale training behavior.
2. Replace `zero_actions` with `torch.randn_like(zero_actions) * 5.0` (a
   deliberately large random action) and rerun. Given the PD-target
   clamping described above, does the robot still fall over in a
   qualitatively different way than the zero-action case, or does the
   joint-limit clamp make "large random action" and "zero action" converge
   to similar failure behavior?
3. Try switching `--device cuda:0` to `--device cuda:1` (or vice versa) and
   compare `lecture20_results.txt`'s `device=` line against what you asked
   for. Confirm the environment actually landed on the GPU you requested
   rather than assuming `--device` took effect.

## Next

[Lecture 21 — Training G1 to walk with PPO](lecture21.md): a real
500-iteration training run of `Isaac-Velocity-Flat-G1-v0` using Isaac Lab's
own `rsl_rl` scripts, what the reward curve actually looks like, and NVIDIA's
own published checkpoint walking, side by side with the untrained env this
lecture just showed collapsing in under a second.
