# Isaac Sim & Omniverse API Tutorial

A from-scratch, run-it-yourself course on the **Isaac Sim / Omniverse Kit / USD /
PhysX** Python API — the layer underneath every Isaac Sim project, robot import,
and RL environment, that most tutorials skip past on the way to something flashier.

Each lecture is one small, runnable Python script plus a markdown writeup. No
lecture depends on GUI clicking-around you can't script, and no lecture depends
on content from a later one. Run them in order, or jump to whichever concept
you're stuck on.

This isn't a paraphrase of the official docs. Several lectures exist because a
real project hit a real bug that the docs don't warn you about — a stage with no
physics scene that plays and simulates *nothing*, a light that provably cannot
reach an enclosed room, a transform write that silently does nothing once physics
is stepping. Where that happened, the lecture says so and shows you the failure
before showing you the fix.

## Who this is for

You've installed Isaac Sim, maybe followed the official "load a robot and drive
it" quickstart, and it worked — but you don't actually know what a `Stage` is,
why some scripts start with `SimulationApp({...})` before anything else, or why
your camera returns nothing until you wait 30 frames. This fills that gap.

## Prerequisites

- A working Isaac Sim install. These lectures were written and verified against
  **Isaac Sim 6.0.1** (Kit's embedded Python is **3.12**). Isaac Sim 5.x is close
  enough that most lectures will run unmodified — see the version note at the
  bottom of each lecture's `.md` for the handful of spots where the API moved
  between major versions.
- No GPU-heavy assets required for Lectures 1-16. Every one of those builds
  its scene from primitive shapes authored directly in the script — nothing
  streams from a content CDN, so nothing there needs a fast connection to run.
  Lectures 17-19 are the exception: a humanoid robot + trained locomotion
  policy and a Jetbot mesh can't be authored from primitives, so those three
  pull assets from Isaac Sim's Nucleus/CDN the first time they run. That
  dependency is flagged in each of those lectures' own `.md` files.
- Comfortable reading Python. No prior Omniverse/USD knowledge assumed — that's
  the whole point of lecture 2.

## How to run a lecture

Isaac Sim ships its own Python interpreter with the whole SDK pre-wired onto
the path — you run scripts *through it*, not through your system Python or a
venv:

```bash
<your-isaac-sim-install>/python.sh lectures/lecture01.py
```

On this machine that's:

```bash
/home/gtu-dsa/robotics/isaacsim-6.0.1/python.sh lectures/lecture01.py
```

Every lecture runs **headless** (no window) by default so it works over SSH and
in CI — that choice is itself explained in lecture 1. Expect the first ~8-10
seconds of any run to be Kit extension startup logged to your terminal; that's
normal and every lecture's `.md` tells you what output to expect after that.

## Curriculum

### Module 1 — Fundamentals (Lectures 1-10)

| # | Lecture | What it teaches |
|---|---------|------------------|
| 1 | [Hello Simulation](lectures/lecture01.md) | `SimulationApp`, the headless app lifecycle, why import order matters |
| 2 | [Stage & USD Basics](lectures/lecture02.md) | Stage / Layer / Prim / Attribute, `DefinePrim`, traversal, saving `.usda` |
| 3 | [Transforms](lectures/lecture03.md) | `XformCommonAPI`, translate/rotate/scale, the rotation-order convention that trips everyone up once |
| 4 | [Composition: References vs Flattening](lectures/lecture04.md) | Referencing an asset into your stage vs `Stage.Export()`, and why one of those can turn a 20 KB file into 200+ MB |
| 5 | [Physics Scene & Rigid Bodies](lectures/lecture05.md) | `UsdPhysics.Scene`, `RigidBodyAPI`/`CollisionAPI`/`MassAPI`, and a documented old finding ("nothing simulates without a scene") that a fresh check on this version proved wrong |
| 6 | [The Simulation Loop](lectures/lecture06.md) | `timeline.play()/pause()/stop()` — three different behaviors, not a toggle — and why `kit.update()` is a bundle (UI+render+physics), not one atomic step |
| 7 | [Cameras & Rendering](lectures/lecture07.md) | `UsdGeom.Camera`, `isaacsim.sensors.camera.Camera`, capturing RGBA, focal length/aperture → field of view, and why the first couple of frames come back `None` |
| 8 | [Lights](lectures/lecture08.md) | `UsdLux` light types, and a worked demo of why a `DomeLight` alone cannot light an enclosed room — and why turning it up doesn't fix that |
| 9 | [Articulations & Joint Drives](lectures/lecture09.md) | `ArticulationRootAPI`, `RevoluteJoint`, `DriveAPI` stiffness/damping, steady-state error under load, and a real `FixedJoint` anchor bug caught by checking rather than assuming |
| 10 | [Capstone](lectures/lecture10.md) | One script: physics + a driven articulated joint + placed lighting + a camera, wired together end to end — and the scene that caught Lecture 09's bug |

### Module 2 — Sensing, Mapping, and Mobile Robots (Lectures 11-19)

| # | Lecture | What it teaches |
|---|---------|------------------|
| 11 | [2D LiDAR](lectures/lecture11.md) | `GenericModelOutput`'s `(x, y, z)` fields are really `(azimuth, elevation, distance)` by default — decoding a real RTX Lidar scan instead of assuming cartesian points |
| 12 | [3D LiDAR](lectures/lecture12.md) | Swapping in a full 3D rotary config and re-verifying Lecture 11's decoding still holds once elevation is no longer always zero |
| 13 | [Camera Parameters Deep Dive](lectures/lecture13.md) | Building the intrinsics matrix from focal length/aperture/resolution and checking it against where a real 3D point actually lands on a rendered image |
| 14 | [Depth Cameras and RGB-D](lectures/lecture14.md) | `distance_to_image_plane` vs `distance_to_camera` — two different depth conventions that only agree at the image center, and which one backprojection needs |
| 15 | [Mapping](lectures/lecture15.md) | Turning one lidar scan into a full occupancy grid — bin every azimuth, keep the nearest return, mark FREE/OCCUPIED/UNKNOWN per cell |
| 16 | [Path Planning](lectures/lecture16.md) | A* over the occupancy grid, with an added obstacle whose blocking effect and detour are both verified in code, not assumed |
| 17 | [Humanoid with a Ready-to-Use Policy (H1)](lectures/lecture17.md) | Driving a trained locomotion policy through Lecture 09's PD-drive machinery, and a real render/physics-decoupling bug that collapses the robot while the script still exits 0 |
| 18 | [Differential-Drive Controller (Jetbot)](lectures/lecture18.md) | `DifferentialController` + wheel odometry — and why calibrating one kinematic constant against straight-line motion alone made turning odometry 1.6x worse |
| 19 | [Module 2 Capstone](lectures/lecture19.md) | Scan → map → plan → drive in one script, verifying the plan against real physics — plus a bug from combining two of this course's own API eras in one file |

### Module 3 — Reinforcement Learning with Isaac Lab (Lectures 20-21)

> **Different toolchain.** Lectures 1-19 run directly against an Isaac Sim
> install's `python.sh`. Module 3 needs a separate **Isaac Lab** install
> (this course used **Isaac Lab 2.2.1**, which pins **Isaac Sim 5.0.0** /
> Python 3.11 — not the 6.0.1 / Python 3.12 pairing every earlier lecture
> used) and runs through `isaaclab.sh -p`, not `python.sh`. See
> [Lecture 20](lectures/lecture20.md) for the exact install/run details.

| # | Lecture | What it teaches |
|---|---------|------------------|
| 20 | [Isaac Lab Environments 101](lectures/lecture20.md) | What `gym.make()` on a registered Isaac Lab task actually hands back — observation/action space shapes, why the action space is unbounded rather than normalized, and a Kit-specific gotcha where stdout silently stops reaching a redirected log file mid-run |
| 21 | [Training G1 to Walk with PPO](lectures/lecture21.md) | A real 500-iteration PPO training run against `Isaac-Velocity-Flat-G1-v0` using Isaac Lab's own `rsl_rl` scripts, a reward curve with a real early plateau before it breaks through, and the trained checkpoint's measured before/after against Lecture 20's untrained baseline (8/8 falls → 0/8) |

## Why this order

Module 1 is fundamentals only — no ROS2 bridge, no RL training loop, no
specific robot import. Those are all built *on top of* what's here, and every
tutorial for them assumes you already have this. If Lectures 1-10 make sense,
the official robot-import and Isaac Lab tutorials will stop feeling like
magic.

Module 2 builds one mobile-robot pipeline end to end: sense the environment
(11-14), turn a scan into a map and a map into a plan (15-16), then actually
drive — first a humanoid on a pretrained policy (17), then a wheeled robot
under direct kinematic control with its own odometry (18), then all four
pieces chained together and checked against real physics rather than trusted
individually (19). Several of these lectures exist specifically because
chaining verified pieces together surfaced a bug none of them showed alone —
that's the point of ending on a capstone instead of stopping once each piece
works in isolation.

Module 3 changes the question. Modules 1-2 hand-author a control loop and
trust it because you wrote every line of it. Isaac Lab's
`ManagerBasedRLEnv` is the same physics/articulation machinery underneath,
now assembled from a declarative config and driven by a policy *learned*
instead of written — so the two lectures here inspect the environment
before trusting it (20), then train a real policy on it and measure the
actual before/after rather than assuming training worked (21). It's short
by design: the point isn't to re-derive PPO, it's to show what changes
(and what doesn't) once the control loop is trained rather than hand-tuned,
using the same "check, don't assume" habit as every earlier module.

## Contributing

Found a spot where a newer Isaac Sim version changed the API out from under a
lecture? Open an issue or a PR — that's exactly the kind of drift this repo
exists to keep documented in one place instead of everyone re-discovering it
independently.

## License

MIT — see [LICENSE](LICENSE). Use any of this in your own docs, courses, or
onboarding material.
