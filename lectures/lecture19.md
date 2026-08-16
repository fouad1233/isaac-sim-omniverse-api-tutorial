# Lecture 19 — Module 2 capstone: scan, map, plan, drive

**Code:** [`lecture19.py`](lecture19.py)

## The one thing this lecture teaches

Lectures 15, 16, and 18 each verified their own piece — a scan produces
a sane occupancy grid, A* finds a sane path over that grid, a
differential-drive controller can be commanded and its odometry checked.
None of them checked whether the *plan* actually works once a real robot
tries to execute it. This lecture chains all four steps in one script —
scan the room, build the map, plan a route, then physically drive a
Jetbot along it — and the final assertion is the one none of the earlier
lectures could make: the robot's simulated pose, not the planner's
printout, ends up within 25cm of the goal. It also documents a real bug
that only shows up once a script *does* combine an old-style sensor
phase with a new-style physics-driving phase — the first time this course
has done that.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture19.py
```

## What you'll see

```
LECTURE: [scan] captured 15152178 beam returns, azimuth range [-180.0, 180.0] deg, distance range [1.70, 4.87] m
LECTURE: [map] grid is 35x45 cells at 0.2m/cell -- 738 free, 113 occupied, 724 unknown
LECTURE: [plan] A* path: 19 cells, 4.10 m, start=(-1.5, 2.0) -> goal=(2.0, 2.0)
LECTURE: [plan] simplified to 9 waypoints: ['(-1.50, 2.10)', '(-0.90, 2.10)', '(-0.30, 1.50)', '(0.90, 1.50)', '(1.10, 1.70)', '(1.30, 1.70)', '(1.50, 1.90)', '(1.90, 1.90)', '(2.10, 2.10)']
LECTURE: [drive] jetbot spawned at (-1.500, 2.000), heading 0.1 deg -- driving toward 8 waypoint(s)
LECTURE: [drive] 1060 physics steps (17.7s sim time)
LECTURE: [drive] ground truth final pose  -> x=2.031 y=2.028 theta=41.7 deg -- 0.042 m from the planned goal (2.0, 2.0)
LECTURE: [drive] odometry-believed pose   -> x=1.799 y=2.299 theta=49.1 deg -- 0.357 m from ground truth (what Lecture 18 already showed to expect from wheel odometry alone)
LECTURE: the plan built from one lidar scan, executed with only wheel velocity commands and no access to ground truth during the drive, actually got the robot to the goal -- verified from the simulated pose, not assumed from the planner's output
```

## Walking through it

**Three phases, two different eras of this course's own API usage, in
one script.** Phase 1 (scan) and Phase 2 (map + plan) are Lecture 16's
code, byte-for-byte: `omni.timeline.get_timeline_interface().play()`,
raw `UsdGeom`/`UsdPhysics` prim authoring, an `omni.replicator.core`
writer accumulating lidar returns — no `PhysicsScene` needed, because
nothing is dynamic yet. Phase 3 (drive) is Lecture 18's code: a fresh
`UsdPhysics.Scene`, a `WheeledRobot`, `SimulationManager.setup_simulation()`,
a plain per-step `apply_wheel_actions()` / `update_app(steps=1)` loop. Both
halves were independently verified in their own lectures. Concatenating
them produced a bug neither one showed on its own.

**The new bug: a lidar left running into a phase that no longer needs
it.** Phase 1's `LidarSensor` and its attached `CaptureWriter` do their
job during the scan and are never touched again — but the first version
of this script also never told them to stop. Phase 3 then builds a
second, independent physics setup: a new `UsdPhysics.Scene`, a new
`WheeledRobot`, a second `timeline.play()` via
`app_utils.play()`. That tore down and rebuilt the physics simulation
view — which is exactly what Lecture 18's identical pattern does too,
safely, in a script that never had a lidar in it. Here, the sensor
(running with `enable_motion_bvh=True`, which ties its RTX render
pipeline into PhysX's bounding-volume tracking) was still attached to
the *old* view when Phase 3's rebuild happened. Its own re-initialization
finished a few ticks into the drive loop, invalidating the Jetbot's
physics tensor view out from under it — not at construction, not at the
first `get_dof_velocities()` call after `play()` (that one worked fine),
but partway through the very first waypoint's control loop:

```
[Warning] [omni.physx.tensors.plugin] All physics information was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.
[Warning] [isaacsim.core.experimental.prims.impl.articulation] Invalid physics simulation view. Articulation (['/World/Jetbot']) will not be initialized
AssertionError: Instance's physics tensor entity is not valid. Play the simulation/timeline to re-initialize it
```

**Isolating it.** A minimal script reproducing just the ingredients —
timeline play/stop with a bare floor, no lidar — did *not* crash, even
with `enable_motion_bvh=True` set. Adding the actual `Lidar` +
`CaptureWriter` from this lecture's Phase 1, left attached, reproduced
the crash on the very first iteration of a stand-in drive loop.
Detaching the writer (`sensor.detach_writer("CaptureWriter")`) between
Phase 1 and Phase 3 made the identical drive loop run clean. That's the
fix this script now uses — one line, placed right after Phase 1 confirms
it captured a scan, with a comment explaining why it's there rather than
leaving a future reader to rediscover this by hitting the same crash.

**Why this didn't show up in Lecture 16 or Lecture 18 individually.**
Lecture 16 never builds a `WheeledRobot` or a `PhysicsScene`, so there's
no physics tensor view for a dangling sensor to race against. Lecture 18
never has a lidar in the first place. The bug is a genuine consequence
of composition, not something either lecture's own verification could
have caught — which is itself the point of building a capstone instead
of trusting that four independently-correct pieces compose correctly.

**The pipeline result, taken at face value.** One scan (15.2M beam
returns from a single rotating 2D lidar over 200 physics steps) produces
a 35×45 occupancy grid; A* finds a 19-cell, 4.10m path around the
pillar; `simplify_path()` collapses it to 8 waypoints; a P-controller
following those waypoints with only wheel-velocity commands (no ground
truth used during driving) lands the robot 0.042m from the planned goal
after 1060 physics steps (17.7s of sim time). The odometry estimate
computed alongside the drive — the same nominal-radius dead-reckoning
Lecture 18 built — drifts to 0.357m from the true final pose, consistent
with what Lecture 18 already showed to expect from wheel odometry alone.
Both numbers are printed from the simulated physics state, not asserted
from the planner's intent.

## Try it yourself

1. Remove the `sensor.detach_writer("CaptureWriter")` line and rerun.
   Confirm you reproduce the exact `AssertionError` above, and note
   *where* in the drive loop it happens — first waypoint, first few
   physics steps in, not at `WheeledRobot()` construction or the initial
   `get_dof_velocities()` call right after `play()`.
2. Instead of detaching the writer, try removing the `/World/Lidar` prim
   entirely after Phase 1 (`stage.RemovePrim("/World/Lidar")`). Does that
   fix it too, or does the writer detachment specifically matter
   independent of whether the sensor prim itself still exists?
3. Set `enable_motion_bvh` to `False` in the `SimulationApp` config (with
   the writer left attached, bug reproduced) and rerun. Does the crash
   still happen? This tells you whether `enable_motion_bvh` is a
   necessary ingredient of the race or just what this lecture happened to
   need for the lidar to scan correctly in the first place.
4. Change `goal_world` to a point on the far side of the pillar from
   `start_world` and rerun. Does `simplify_path()` still produce a
   sensible small waypoint count, and does the P-controller still reach
   the (harder) goal within `MAX_STEPS_PER_WAYPOINT` per leg?

## Next

That's Module 2. From here: extending this stack to a dynamic obstacle,
swapping the hand-rolled A* for a library planner, or replacing wheel
odometry with something that doesn't drift (visual or lidar-based
localization) are the natural next steps this capstone sets up but
doesn't take.
