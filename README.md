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
- No GPU-heavy assets required. Every lecture builds its scene from primitive
  shapes authored directly in the script — nothing streams from a content CDN,
  so nothing here needs a fast connection to run.
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

| # | Lecture | What it teaches |
|---|---------|------------------|
| 1 | [Hello Simulation](lectures/lecture01.md) | `SimulationApp`, the headless app lifecycle, why import order matters |
| 2 | [Stage & USD Basics](lectures/lecture02.md) | Stage / Layer / Prim / Attribute, `DefinePrim`, traversal, saving `.usda` |
| 3 | [Transforms](lectures/lecture03.md) | `XformCommonAPI`, translate/rotate/scale, the rotation-order convention that trips everyone up once |
| 4 | [Composition: References vs Flattening](lectures/lecture04.md) | Referencing an asset into your stage vs `Stage.Export()`, and why one of those can turn a 20 KB file into 200+ MB |
| 5 | [Physics Scene & Rigid Bodies](lectures/lecture05.md) | `UsdPhysics.Scene`, why nothing falls without one, `RigidBodyAPI`/`CollisionAPI`/`MassAPI` |
| 6 | [The Simulation Loop](lectures/lecture06.md) | `timeline.play()`, `kit.update()`, physics steps vs render frames, why "settle time" is a real thing to budget for |
| 7 | [Cameras & Rendering](lectures/lecture07.md) | `UsdGeom.Camera`, `isaacsim.sensors.camera.Camera`, capturing RGBA, focal length/aperture → field of view |
| 8 | [Lights](lectures/lecture08.md) | `UsdLux` light types, and a worked demo of why a `DomeLight` alone cannot light an enclosed room |
| 9 | [Articulations & Joint Drives](lectures/lecture09.md) | Building a jointed body, `DriveAPI` stiffness/damping, commanding and verifying convergence |
| 10 | [Capstone](lectures/lecture10.md) | One script: stage + physics + light + camera + a falling body, wired together end to end |

## Why these specific ten

Fundamentals only — no ROS2 bridge, no RL training loop, no specific robot
import. Those are all built *on top of* what's here, and every tutorial for
them assumes you already have this. If lectures 1-10 make sense, the official
robot-import and Isaac Lab tutorials will stop feeling like magic.

## Contributing

Found a spot where a newer Isaac Sim version changed the API out from under a
lecture? Open an issue or a PR — that's exactly the kind of drift this repo
exists to keep documented in one place instead of everyone re-discovering it
independently.

## License

MIT — see [LICENSE](LICENSE). Use any of this in your own docs, courses, or
onboarding material.
