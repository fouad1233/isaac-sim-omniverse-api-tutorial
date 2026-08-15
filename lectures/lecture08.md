# Lecture 08 — Lights and Why Dome Lights Don't Reach Interiors

**Code:** [`lecture08.py`](lecture08.py)

## The one thing this lecture teaches

A `DomeLight` models an infinitely distant environment — the sky, in the
usual mental model — and it lights every surface that can "see" that sky
in an unobstructed straight line. Seal a room around a camera and every
interior surface's line of sight to that sky is blocked by the room's own
walls, so the dome contributes nothing inside, no matter how bright you
make it. This is the actual, generalizable version of a real bug: a
project this course grew out of had dim camera footage inside an enclosed
space, and turning up exposure/gain didn't fix it, because the problem was
never brightness — it was that no light was placed where the camera could
actually see its effect.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture08.py
```

## What you'll see

```
LECTURE: mean RGB brightness (0-255 scale) seen by a camera INSIDE the sealed room
LECTURE:   A_dome_1000                      ->   0.00
LECTURE:   B_dome_50000                     ->   0.00
LECTURE:   C_dome_1000_plus_interior        ->  30.96

LECTURE: 50x brighter dome changed interior brightness by +0.00 (barely, if at all -- intensity wasn't the problem)
LECTURE: one modest interior light changed it by +30.96 (placement was)
```

Three saved PNGs tell the same story visually:
[`output_lecture08_a_dome_only.png`](output_lecture08_a_dome_only.png) and
[`output_lecture08_b_dome_bright.png`](output_lecture08_b_dome_bright.png)
are both flat black — not "dim," literally `0.00` mean brightness, 50x
intensity difference between them and no visible difference at all.
[`output_lecture08_c_interior_light.png`](output_lecture08_c_interior_light.png)
shows the interior light itself as a glowing spot and the target cube's
near face dimly visible next to it — the *only* case with anything to see.

## Walking through it

**The room is sealed on purpose — six walls, no gaps.** `build_room()`
makes a floor, ceiling, and four walls out of unit cubes stretched with
`SetScale`, the same `XformCommonAPI` from Lecture 03 with the one
operation it hadn't needed yet. "Sealed" isn't incidental to this
lecture's result, it's the entire mechanism: a `DomeLight` in Omniverse's
real-time renderer contributes to a surface point based on that point's
visibility toward the environment, and there is no visibility toward
"the sky" from anywhere inside a fully closed box.

**Case A vs. Case B is the part worth sitting with.** Same dome, same
room, same camera — the only change is `dome.GetIntensityAttr().Set(...)`
from `1000.0` to `50000.0`, a 50x jump. The result: `0.00` both times, to
the decimal place printed. If your instinct on seeing a dark render is "turn
the light up," this is the case where that instinct does nothing, because
the light was never reaching the surface in the first place — not weakly,
not partially, not at all. No amount of intensity fixes an occlusion
problem, because intensity was never the variable that mattered.

**Case C changes exactly one thing: where a light physically sits.** The
dome goes back to its original `1000.0` — no change there — and a single
`UsdLux.SphereLight` gets placed *inside* the room, near the camera, the
same general idea as mounting a light where the thing that needs to see is
looking. Mean brightness jumps from `0.00` to `30.96`. Not because
anything got "brighter" in the abstract — one specific point in space
gained a light that has an unobstructed path to the surfaces the camera
can actually see.

**This is a general instance of a specific real bug.** A project this
course is written alongside had a navigation camera producing dim footage
inside a building interior. The fix wasn't exposure, gain, or tone
mapping — it was that the scene's light wasn't physically positioned
anywhere the camera's view could reach, for exactly the occlusion reason
demonstrated above. This lecture is that finding, stripped down to the
smallest scene that reproduces it and re-verified from scratch rather than
retold from memory — the same standard every other lecture in this course
holds itself to.

## Try it yourself

1. Leave a gap in one wall — shrink `WallFar`'s scale so it doesn't quite
   reach the ceiling — and rerun. Does dome light leak in through the gap?
   How much of a gap does it take before `A_dome_1000`'s brightness moves
   off `0.00`?
2. Move `InteriorLight`'s position to just outside the sealed room instead
   of inside it (e.g. `(-4.0, 0.0, 1.5)`, past `WallNear`). Does the room
   go dark again, the same way the dome did? Same mechanism, different
   light type.
3. Try a `UsdLux.RectLight` instead of a `SphereLight` for the interior
   light, sized to cover part of the ceiling, as a stand-in for a "ceiling
   panel" light source. Compare its brightness contribution and the shape
   of the light it casts against the point-like `SphereLight` used here.

## Next

[Lecture 09 — Articulations and Joint Drives](lecture09.md): everything up
to here has been rigid, single-piece geometry. This lecture builds
something with a moving joint — a hinge that can be commanded to a target
angle and watched as it gets there.
