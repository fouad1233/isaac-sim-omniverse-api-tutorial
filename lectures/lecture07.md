# Lecture 07 — Cameras and Rendering

**Code:** [`lecture07.py`](lecture07.py)

## The one thing this lecture teaches

A `UsdGeom.Camera` is, on its own, just a position and two lens numbers —
it cannot hand you a single pixel. Getting an actual image out of Isaac Sim
needs a second thing entirely: a render pipeline attached to that camera,
which `isaacsim.sensors.camera.Camera` wraps for you. And that render
pipeline has its own version of Lecture 06's lesson — calling `update()`
doesn't mean the data you asked for is ready yet, except this time it's
pixels, not physics.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture07.py
```

## What you'll see

```
LECTURE: UsdGeom.Camera defaults -- focalLength=50.0 mm, horizontalAperture=20.955 mm
LECTURE: step 2 -- first step get_rgba() returned real pixels (shape=(240, 320, 4))
LECTURE: steps 0..1 all returned None/empty -- the render pipeline wasn't ready yet, same 'update() doesn't mean data is ready' lesson from Lecture 06, now for pixels instead of physics.

LECTURE: horizontal FOV from focalLength/aperture formula = 23.6702 deg
LECTURE: camera.get_horizontal_fov() = 0.4131 rad = 23.6702 deg (matches the formula: True)

LECTURE: saved <repo>/lectures/output_lecture07_top_down.png
LECTURE: red-pixel centroid=(159.5, 119.5), image center=(160.0, 120.0) -- 28898 red pixels found
```

And `output_lecture07_top_down.png` itself: a red square dead-center on a
black background — a top-down shot of a cube, from a camera aimed straight
down at it.

## Walking through it

**Two things had to exist before any pixel could exist.** The
`UsdGeom.Camera` prim (`cam_geom`) is the USD side — it's where the camera
*is* and what its lens numbers are, and it's real the instant it's
authored, same as any other prim. But nothing about a USD schema knows how
to trace rays or produce an image. That's `isaacsim.sensors.camera.Camera`
— a wrapper that attaches an actual render pipeline (a "render product," in
Omniverse terms) to the prim path you give it. Skip the wrapper and you
have a well-defined camera that no code can get an image out of.

**The aiming needed zero rotation math, on purpose.** Lecture 03 spent a
whole lecture establishing that cameras (and lights) look down local `-Z`
by default. This scene puts the camera on the `+Z` axis, straight above
the cube, on a `z`-up stage — so local `-Z` already points straight down at
it, no `SetRotate` call needed at all. That's not an accident of this
scene; it's Lecture 03's finding paying off. Aiming a camera at an angle is
exactly the same rotation problem as aiming the headlight in that lecture
— nothing new to learn here, so this lecture didn't make you re-derive it.

**`get_rgba()` returned `None` for the first couple of steps, not an
image.** This is the same shape of problem Lecture 06 closed on: a render
pipeline has its own internal "not ready yet" that has nothing to do with
whether `kit.update()` returned successfully. Here it shows up as literally
`None` for a couple of steps before the first real frame — call
`get_rgba()` too early and you get nothing, not a black image, not an
error, just `None`. A script that grabs frame 0 unconditionally would
silently work with no image at all. This is a smaller version of a real
problem: RTX rendering can take several frames to *converge* even after it
starts returning data (denoising, temporal accumulation) — invisible in
this lecture's flatly-lit, single-color scene, but very real in busier
ones. Lecture 08 runs into a version of this for real.

**The horizontal FOV is derived, not stored.** `UsdGeom.Camera` only ever
stores `focalLength` and `horizontalAperture` (both in millimeters, a
holdover from real camera lens conventions). Field of view is computed from
them: `fov = 2 * atan(aperture / (2 * focalLength))`. The script computes
that by hand and checks it against `camera.get_horizontal_fov()` — they
match, once you remember the wrapper's version comes back in **radians**,
not degrees. Two different units agreeing after conversion is a better
confirmation than either number alone.

**The centroid check is the real test, not the picture.** A `(159.5,
119.5)` centroid against a `(160.0, 120.0)` image center — half a pixel off
purely from resolution parity (320/240 are even numbers; a single pixel
column can't sit exactly on their midpoint) — is what "the camera is
pointed where geometry says it should be" looks like as a number instead of
a guess from eyeballing an image.

## Try it yourself

1. Read `camera.get_rgba()` at `step == 0` specifically and check what you
   get back before the loop's `first_valid_step is None` guard would have
   skipped it. Confirm it's really `None`, not an all-black or all-zero
   array — those are different failure modes with different causes.
2. Change `resolution=(320, 240)` to something odd like `(321, 241)`. Does
   the centroid land exactly on the new image center?
3. Move the camera off-axis — translate it to `(3.0, 0.0, 4.0)` instead of
   `(0.0, 0.0, 5.0)`, still with zero rotation. The cube won't be centered
   anymore, and predicting *where* it'll land (hint: which direction is now
   "up" in the frame, and which way did the camera not move) is a good
   check of whether Lecture 03's rotation convention actually stuck.

## Next

[Lecture 08 — Lights and Why Dome Lights Don't Reach Interiors](lecture08.md):
this lecture's cube was lit by a single, generously bright distant light
with nothing in the way. Real scenes have walls, and a light that
illuminates *the whole scene* on paper can still leave *what your camera
actually sees* dark, for a very specific, verifiable reason.
