# Lecture 05 — Physics Scene & Rigid Bodies

**Code:** [`lecture05.py`](lecture05.py)

## The one thing this lecture teaches

Geometry and physics are separate opt-ins in USD. A `Cube` is just a shape
until you apply `UsdPhysics.CollisionAPI` (now it can be hit) and
`UsdPhysics.RigidBodyAPI` (now PhysX simulates it). There's a third piece,
`UsdPhysics.Scene`, that configures the *world* those bodies fall through —
gravity direction and magnitude, mainly — and this lecture exists because
what happens when you skip it is more subtle, and more version-dependent,
than it looks.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture05.py
```

## What you'll see

```
LECTURE: height (m) over time -- three scenes, same starting height 2.0 m
LECTURE:  step  no scene (implicit)   scene, g=9.81    scene, g=2.0
LECTURE:     0               2.0000          2.0000          2.0000
LECTURE:    10               1.7874          1.7874          1.9567
LECTURE:    20               1.3106          1.3106          1.8594
LECTURE:    30               0.5612          0.5612          1.7067
LECTURE:    40               0.1000          0.1000          1.4983
LECTURE:    60               0.1000          0.1000          0.9150
LECTURE:    80               0.1000          0.1000          0.1094
LECTURE:   100               0.1000          0.1000          0.1000

LECTURE: implicit (no scene) trace exactly matches explicit g=9.81 trace: True
LECTURE: (that's not a coincidence -- see lecture05.md)
```

## Walking through it

**Three separate opt-ins, and it's worth knowing which is which.**
`UsdPhysics.CollisionAPI` gives a prim a collision shape — something else
can hit it. `UsdPhysics.RigidBodyAPI` is what makes PhysX actually
integrate a body's motion each step — without it, a prim with collision is
just static geometry, like the ground cube here (collision, no rigid body,
never moves, never falls, exactly what a floor should do).
`UsdPhysics.MassAPI` sets an explicit mass; skip it and PhysX estimates one
from the collision geometry's volume and a default density — a real number
either way, just not one you chose.

**Now the part this lecture is actually about.** An older, previously
verified finding from a real project on an earlier Isaac Sim version was:
*no `UsdPhysics.Scene` prim on the stage means nothing simulates at all* —
a fully-configured rigid body just hangs frozen in the air forever, no
error, nothing. That was true, and cost real debugging time to root-cause
when it first happened.

It is **not** what this script's first column shows. `none_trace` — the
run with zero `UsdPhysics.Scene` prims anywhere on the stage — falls
identically to a run with an explicitly authored one at Earth gravity. Not
"falls too, coincidentally close" — identical to four decimal places, at
every checkpoint. That match is the tell: newer `omni.physx` (confirmed
here on Isaac Sim 6.0.1) creates an **implicit fallback physics scene**
when you don't author your own, and it defaults to ordinary Earth gravity,
9.81 m/s² straight down.

**Which is exactly why you should still always author your own.** Look at
the third column: `gravity_magnitude=2.0` falls visibly slower — still at
height 0.91 m by the step where the other two have already landed at
0.10 m. That number is only reachable by authoring `UsdPhysics.Scene`
yourself. The implicit fallback gives you *a* physics world, but not one
you control — no custom gravity, no non-Earth environment (weightless,
lunar 1.62 m/s², whatever your scene calls for), and no guarantee its
defaults stay the same in the next Isaac Sim release, because nothing
about "there is a fallback and here is what it defaults to" is a
documented, stable contract the way an explicitly authored prim is.

**This is also a lesson about trusting old notes over a fresh check.**
The "nothing simulates without a scene" claim above wasn't invented for
this lecture — it's a real, previously-verified finding from a real
project, written down in good faith at the time. It just isn't true
anymore, on this version, and the only way to find that out was to build
exactly this comparison and run it, rather than to keep repeating the
older claim because it used to check out. Software you don't control the
release cadence of is not a fact you get to memorize once.

## Try it yourself

1. Change `gravity_direction` to `(1.0, 0.0, 0.0)` in an authored scene
   (still magnitude 9.81) and rerun with a box that starts away from any
   wall. Confirm it accelerates sideways, not down — "gravity" is just a
   configured direction and magnitude, nothing more special than that.
2. Set `gravity_magnitude` to `0.0`. Does the box move at all? (It
   shouldn't — but check what "still" actually looks like: exactly frozen
   at 2.0000, the same way the pre-fix frozen case in the older finding
   looked. Same symptom, opposite cause — worth being able to tell apart.)
3. Try this exact comparison against whatever Isaac Sim version you
   actually have installed. If your `none_trace` differs from your
   `earth_trace`, you've just found a version where the old finding still
   holds — which is itself useful to know, and not something this lecture
   can tell you in advance for your setup.

## Next

[Lecture 06 — The Simulation Loop](lecture06.md): this lecture's
`kit.update()` loop stepped physics without a second thought about timing.
The next one is about why that "just call update() in a loop" pattern
sometimes isn't enough — specifically for rendering, not physics.
