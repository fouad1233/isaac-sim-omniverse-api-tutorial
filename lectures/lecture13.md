# Lecture 13 — Camera parameters deep dive

**Code:** [`lecture13.py`](lecture13.py)

## The one thing this lecture teaches

Lecture 07 introduced `focalLength` and `horizontalAperture` just far
enough to derive horizontal FOV. Those same numbers, plus resolution,
build the intrinsics matrix every perception pipeline downstream of this
camera actually relies on — and this lecture checks that matrix against
where a real 3D point lands on a real rendered image, not just against a
formula. Along the way it finds a real quirk in the SDK's own
`get_vertical_fov()`, and a real render-pipeline latency bug in its own
first draft — both found the same way everything else in this course gets
found: run it, check the number, don't assume.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture13.py
```

## What you'll see

```
LECTURE: baseline render -- 28898 red pixels, centroid=(159.5, 119.5)
LECTURE: focalLength=5.0mm horizontalAperture=2.095mm verticalAperture=1.572mm resolution=320x240
LECTURE: hand-built intrinsics matrix: ...
LECTURE: camera.get_intrinsics_matrix(): ...
LECTURE: match = True
LECTURE: predicted pixel coords of cube center (0,0,0) = (160.0, 120.0)
LECTURE: measured red-pixel centroid                    = (159.5, 119.5)
LECTURE: prediction error = 0.71 px (should be small -- ...)
LECTURE: horizontal FOV -- trig formula=23.6702deg api=23.6702deg (match: True)
LECTURE: vertical FOV   -- trig formula=17.8634deg api=17.7526deg (match: False)
LECTURE: get_vertical_fov()'s actual formula is hfov * height/width, not atan(verticalAperture/2*focalLength): 23.6702 * 240/320 = 17.7526deg -- matches the api value, not the trig formula
LECTURE: default clippingRange = (near=1.0, far=1000000.0)
LECTURE: clippingRange=(0.1, 20.0)  -- 28899 red pixels (cube should be visible)
LECTURE: clippingRange=(0.1, 4.0)   -- 0 red pixels (far plane now in front of the cube -- should be 0)
LECTURE: clippingRange=(6.0, 20.0)  -- 0 red pixels (near plane now behind the cube from the camera's side -- should be 0)
LECTURE: clippingRange=(0.1, 20.0)  -- 28898 red pixels (restored -- confirms the cube itself was never the problem)
```

## Walking through it

**The intrinsics matrix is four numbers you already had, not a mystery
box.** `fx = width * focalLength / horizontalAperture`, `fy = height *
focalLength / verticalAperture`, `cx = width/2`, `cy = height/2`. Building
that matrix by hand and comparing it to `camera.get_intrinsics_matrix()`
confirms the formula — but matching a formula isn't the real test. The real
test is `get_image_coords_from_world_points()` correctly predicting where
the cube's actual world-space center, `(0, 0, 0)`, lands in the rendered
image: `(160.0, 120.0)` predicted against `(159.5, 119.5)` measured by
counting red pixels — the same centroid technique Lecture 07 used, now
cross-checked against a projection computed *before* the pixels existed at
all, not after.

**`get_vertical_fov()` doesn't use `verticalAperture`.** The trig formula
that correctly reproduces `get_horizontal_fov()` — `2 * atan(aperture / (2
* focalLength))` — does *not* reproduce `get_vertical_fov()`; it's off by
about a tenth of a degree, small enough to look like rounding error and
easy to wave away, except it isn't. Reading the SDK source
(`isaacsim/sensors/camera/camera.py`, `extsDeprecated/isaacsim.sensors.camera`)
shows why: `get_vertical_fov()` is literally `get_horizontal_fov() *
(height / width)` — a linear rescale of the horizontal *angle* by the
resolution's aspect ratio, not an independent trig computation from
`verticalAperture` at all. On this camera the two aperture numbers happen
to share the same ratio as the resolution (`2.095/1.572 ≈ 320/240 ≈
1.333`, thanks to `set_resolution`'s `maintain_square_pixels` default), so
the approximation is close but not exact — `17.7526°` from the SDK's
formula versus `17.8634°` from the geometrically correct one. If you ever
set `verticalAperture` to something that does *not* match the resolution's
aspect ratio, this gap would grow far past a rounding-error's worth, and
`get_vertical_fov()` would report a number with no real relationship to
what the sensor is actually capturing vertically. Trust the aperture-based
formula for vertical FOV, not this method, until/unless it's fixed upstream.

**The first version of the `clippingRange` test lied, and the fix is the
same lesson Lecture 06 and 07 already taught, in a new spot.** Calling
`set_clipping_range()` then immediately grabbing the first non-empty
`get_rgba()` — exactly the pattern Lecture 07 established for the
*startup* frame — silently returned a frame still influenced by the
*previous* setting, because the render pipeline has its own buffering
independent of whether a frame is merely non-empty. Every result was one
step behind: the far-clip test that should have shown `0` red pixels
showed the unclipped count instead, and the *next* test — supposedly
testing near-clip — actually showed the previous (correct) far-clip
result. The fix isn't a different API call, it's not trusting the first
available frame after a change: run a fixed number of `update()` calls and
keep only the *last* one. Once that's in place, the far plane at `4.0`
(the cube sits roughly 4.5-5.5m away) correctly clips the cube to `0`
pixels, the near plane at `6.0` correctly clips it too (the whole cube is
nearer than that), and restoring `(0.1, 20.0)` brings it right back —
proof the cube itself was never the problem, only when to trust a frame.

## Try it yourself

1. Change `verticalAperture` directly (`camera.set_vertical_aperture(4.0,
   maintain_square_pixels=False)`) so it no longer shares the resolution's
   aspect ratio, and print both vertical FOV formulas again. How far apart
   do they get now, compared to the ~0.11° gap this lecture found?
2. Lower `render_after_change`'s `settle_steps` from `20` down toward `1`
   and rerun the clipping tests. At what point do you start seeing the
   same one-step-behind result this lecture's first draft had?
3. Move the cube to `(1.0, 1.0, 0.0)` instead of the origin, and predict
   its new image coordinates with `get_image_coords_from_world_points()`
   *before* rendering. Does the prediction still land within about a pixel
   of the measured centroid?

## Next

[Lecture 14 — Depth cameras and RGB-D](lecture14.md): the same intrinsics
matrix this lecture built, used to turn a depth image back into 3D points
instead of the other way around.
