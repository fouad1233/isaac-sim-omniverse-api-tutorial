# Lecture 03 — Transforms

**Code:** [`lecture03.py`](lecture03.py)

## The one thing this lecture teaches

Position and orientation live on a prim through **xformOps** — a small
ordered stack of operations (translate, rotate, scale, ...) that gets
composed into a 4×4 matrix. `UsdGeom.XformCommonAPI` is the wrapper that
hides that stack behind three calls. The part that isn't obvious from the
API surface, and that this lecture proves rather than just states, is
**which order the rotation angles compose in** — get it backwards and a
camera or a light points somewhere other than where you told it to.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture03.py
```

## What you'll see

```
LECTURE: world matrix after translate=(1,2,0.5), rotateZ=90:
LECTURE:   [0.0, 1.0, 0.0, 0.0]
LECTURE:   [-1.0, 0.0, 0.0, 0.0]
LECTURE:   [0.0, 0.0, 1.0, 0.0]
LECTURE:   [1.0, 2.0, 0.5, 1.0]

LECTURE: proving R = Rz . Ry . Rx by comparing against USD's own math
LECTURE:   rotateXYZ=(0.0, -90.0, 0.0)  USD forward=[ 1.  0. -0.]  hand-computed Rz.Ry.Rx forward=[ 1.  0. -0.]  match=True
LECTURE:   rotateXYZ=(90.0, 0.0, 0.0)  USD forward=[ 0.  1. -0.]  hand-computed Rz.Ry.Rx forward=[ 0.  1. -0.]  match=True
LECTURE:   rotateXYZ=(30.0, 45.0, 60.0)  USD forward=[-0.739 -0.28  -0.612]  hand-computed Rz.Ry.Rx forward=[-0.739 -0.28  -0.612]  match=True

LECTURE: all cases matched -- R = Rz(rz) . Ry(ry) . Rx(rx) is confirmed, not assumed
LECTURE: holding the window open for 5s -- look at it now
```

If any line said `match=False`, the `assert` right after it would have
crashed the script — nothing here is printed on faith. If you're running
with a real display attached, the window also shows a small orange box
sitting at `(1, 2, 0.5)`, rotated 90° about Z, for five seconds before the
script exits.

## Walking through it

**This lecture uses `ctx.new_stage()`, not the raw `Usd.Stage` API Lecture 2
taught, and that's not a stylistic swap.** Lecture 2 built a Stage with
`Usd.Stage.CreateNew()` on purpose, to show that a Stage exists
independently of any viewport. That independence is exactly the wrong
property here: this lecture runs with `headless=False` so you can watch a
box actually rotate, and a Stage built with the raw API is never shown by
the GUI window — the window only ever renders whatever
`omni.usd.get_context()` holds, which is a different object entirely.
`ctx.new_stage()` creates the Stage *inside* the context to begin with, so
`/World/Thing` and its box show up the moment they're defined, no extra
step required. (An earlier version of this lecture used
`Usd.Stage.CreateInMemory()`, like Lecture 2's file-backed stage but never
touching disk — same disconnect, same fix.) The `Box` under `/World/Thing`
is new for the same reason: an `Xform` alone has no geometry, so even with
the viewport connected there was nothing shaped to actually look at while
the matrix math below was proving itself in the terminal.

**`XformCommonAPI` hides an ordered op stack.** `SetTranslate`, `SetRotate`,
`SetScale` don't write to three independent slots — they author `xformOp:
translate`, `xformOp:rotateXYZ`, `xformOp:scale` and an `xformOpOrder` that
applies them translate-then-rotate-then-scale. You can inspect this
directly: `thing.GetPrim().GetAttribute("xformOpOrder").Get()` after
running the first part of the script. The common API exists specifically so
you don't have to think about op ordering for the 95% case — but it's
useful to know the stack is there, because lecture 9's articulated joints
and lecture 4's referenced assets both build on the same mechanism.

**The rotation tuple is `(rotateX, rotateY, rotateZ)` in *degrees*, and the
order it composes in is the genuinely non-obvious part.** There are two
plausible readings of "rotate by X, then Y, then Z":

1. Compose the matrices in the order written, left to right applied to a
   vector on the right: `R = Rz · Ry · Rx`, then `v' = R v`.
2. Apply X first to the vector, then rotate the *result* about Y, then
   rotate *that* result about Z: `v' = Rz (Ry (Rx v))`.

These look the same and are not — matrix multiplication doesn't commute,
so `Rz · Ry · Rx` and doing the rotations "in tuple order, sequentially" are
only the same statement if you already believe #1. (They happen to expand
to the identical expression algebraically once you write it out — the
confusion is real but the two readings above are actually the same
formula. The bug in practice comes from a *third*, wrong instinct: assuming
each axis rotates around the WORLD axes independently and order doesn't
matter. It very much does — swap `(90, 0, 0)` and `(0, 0, 90)→(90,0,0)`
order in your head and you'll get different answers, because after the
first rotation the "local" axes have moved.)

The script sidesteps arguing about it and just **checks**: build `Rz(rz) ·
Ry(ry) · Rx(rx)` by hand in plain numpy, ask USD for the real
`ComputeLocalToWorldTransform()` of a prim with the same `rotateXYZ` values,
apply both to the same vector, and compare. Three test cases, all
axes-at-once included so nothing lucky can happen — all three agree.

**Why "local -Z" as the test vector, specifically.** It's not arbitrary —
`UsdGeom.Camera` and every `UsdLux` light type point down their own local
**-Z** by default. That's the direction you actually care about when you
aim a camera in [lecture 7](lecture07.md) or a light in
[lecture 8](lecture08.md): "what world direction does this thing end up
facing, given the rotate tuple I set?" The first test case,
`rotateXYZ=(0, -90, 0)`, is not a random choice either — it's the exact
rotation used later in this course to aim a light down the `+X` axis. If
you ever need to point something down `+X` and it's currently pointing
along local `-Z`, `(0, -90, 0)` is the tuple, and now you've seen why.

## Try it yourself

1. Change the first test case to `(0, 90, 0)` (positive instead of
   negative) and predict the resulting forward vector before running.
   Which world axis does local `-Z` end up facing, and is it the one you
   expected?
2. `XformCommonAPI` also has a `SetRotate(value, rotationOrder)` overload
   accepting orders other than XYZ (e.g. `UsdGeom.XformCommonAPI.RotationOrderZYX`).
   Pick one, redo the hand-computed matrix with the axes multiplied in the
   matching order, and confirm it still matches USD's answer — this proves
   the order is a real, respected parameter, not a fixed convention.
3. Reduce the rotate-composition proof to only translate + scale (no
   rotate at all) and inspect `xformOpOrder` — confirm `rotateXYZ` is
   simply absent from the stack when you never call `SetRotate`, rather
   than present with a zero value.

## Next

[Lecture 04 — Composition: References vs Flattening](lecture04.md): now
that one prim can have geometry and a transform, how do you bring in
*someone else's* whole asset — a table, a robot, an environment — without
copying its data into your file?
