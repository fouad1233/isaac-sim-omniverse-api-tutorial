# Lecture 10 — Capstone

**Code:** [`lecture10.py`](lecture10.py)

## The one thing this lecture teaches

Nothing new, on purpose. Every API in this script is from Lectures 01–09 —
a physics scene, a floor, a driven articulated joint, a light placed where
it can actually do something, a camera aimed and read the same
disciplined way every earlier lecture insisted on. The only thing this
lecture adds is combining them in one scene and letting the seams show —
including one real bug that combining them is what actually caught.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture10.py
```

## What you'll see

```
LECTURE: capstone scene settled.
LECTURE: arm driven to 60.0 deg, actual final angle = 59.963 deg
LECTURE: saved frame mean brightness = 153.03 (not 0.00 -- the light is doing real work, per Lecture 08)
LECTURE: saved <repo>/lectures/output_lecture10_capstone.png
```

And [`output_lecture10_capstone.png`](output_lecture10_capstone.png): a
blue base sitting on a gray pedestal, an orange arm swung up to about 60°
from vertical, clearly lit against a visible floor and background — not
the flat black Lecture 08 produced when the only light was a dome outside
a sealed room.

## Walking through it

**One line per earlier lecture, present in this scene:** `UsdPhysics.Scene`
with real gravity (05); `timeline.play()` plus a `kit.update()` loop,
worked exactly like Lecture 06 established, not the `World` wrapper this
course deliberately never introduced; a `UsdGeom.Camera` read through
`isaacsim.sensors.camera.Camera`, with the exact same "`get_rgba()` might
still be `None`" handling Lecture 07 needed (07); a `SphereLight` placed
where the camera's view actually reaches, backed by a deliberately dim
fill dome rather than relied on alone (08); and the one-fixed-base,
one-arm, one-revolute-joint, one-drive pattern from Lecture 09, this time
finishing at 59.963° against a 60° target instead of 44.943° against 45° —
same few-tenths-of-a-degree steady-state gap from gravity loading a finite
stiffness, at a different commanded angle. Lecture 04 — composition and
references — is the one absence, and on purpose: it's a file-organization
technique, not a runtime behavior a single self-contained scene needs to
demonstrate.

**The pedestal is decoration, and staying honest about that matters.**
`/World/Pedestal` never gets `CollisionAPI` or `RigidBodyAPI` — it's a
static mesh with no physics representation at all, there purely so the
floating articulation base doesn't look like it's floating. Nothing in
this scene's dynamics would change if it were deleted. Lecture 05 spent an
entire lecture on "geometry and physics are separate opt-ins"; this is
that fact used deliberately, not just described.

**The floor is what caught a real bug in Lecture 09.** Writing this
capstone put a floor collider at `z = 0` into the same kind of scene
Lecture 09 built without one — and the arm stopped reaching anywhere near
its target. Chasing that down (see the updated
[Lecture 09](lecture09.md#walking-through-it)) found a `FixedJoint` with
an unset local anchor, defaulting to the world origin instead of to
`Base`'s actual position, silently dragging `Base` down to `z = 0` the
moment physics started. Lecture 09's own scene never had anything at
`z = 0` to collide with, so the bug was invisible there — measurable only
by checking `Base`'s literal position, which nothing in that lecture's
printed output did. This capstone's floor is what turned a silent,
still-technically-working bug into a visibly broken one, which is a
genuinely useful thing for a capstone to do: combining working pieces is
exactly how a bug that one piece's own tests don't exercise gets found.

**The final angle check and the brightness check are both real
assertions, not decoration.** `final_angle` is read the same
transform-based way Lecture 09 established, and it's printed precisely so
you can see it's `59.963`, not exactly `60`, and know why (Lecture 09,
again). `mean_brightness` being `153.03` and not `0.00` is the direct,
numeric version of Lecture 08's finding — proof the key light is doing
real work in this frame, not just present in the scene definition.

## Try it yourself

1. Delete the `fill_dome` light entirely and rerun. How much does
   `mean_brightness` drop? Compare that drop to Lecture 08's dome-only
   numbers — is a dim fill dome contributing much at all here, or is the
   `SphereLight` still doing nearly all the work?
2. Reintroduce Lecture 09's original bug on purpose — comment out the
   `fixed.CreateLocalPos0Attr(...)` line — and rerun. Does `final_angle`
   fail the same way it did during development, or does it fail
   differently now that a floor is actually in the scene to collide with?
3. Change `TARGET_DEG` to something well past what's physically reasonable
   for this arm, like `179.0`. Does it still converge, and does the
   steady-state gap from the target grow, shrink, or stay about the same?

## Where to go from here

Ten lectures, each verified by actually running it, not by reading the
docs and hoping. The official Isaac Sim tutorials, the ROS2 bridge, Isaac
Lab's RL environments, and real robot-import workflows all build on
exactly this layer — `Stage`, `Prim`, `UsdPhysics`, the timeline, cameras,
lights, articulations. None of that should feel like magic anymore.
