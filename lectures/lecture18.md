# Lecture 18 — Differential-drive controller and wheel odometry (Jetbot)

**Code:** [`lecture18.py`](lecture18.py)

## The one thing this lecture teaches

`DifferentialController` converts a `[linear speed, angular speed]`
command into `[left, right]` wheel angular velocities with one line of
algebra, and `WheeledRobot.apply_wheel_actions()` feeds those straight
into velocity drives — the same PD-drive machinery as every other
lecture in this module, just in velocity mode instead of position mode.
Running that same algebra *backwards* — wheel motion in, estimated pose
out — is wheel odometry, and this lecture calibrates one of its
constants against the real simulated robot, expecting the calibration to
help. It doesn't. It makes things worse, and understanding exactly why
is the actual lesson.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture18.py
```

## What you'll see

```
LECTURE: jetbot dof_names=['left_wheel_joint', 'right_wheel_joint'], resolved wheel_dof_indices=[0, 1]
LECTURE: calibration -- commanded 0.1 m/s for 3.00s, naive implied distance=0.3000 m, actually measured=0.3369 m (0.1123 m/s, +12.3% vs commanded)
LECTURE: nominal wheel_radius=0.0300 m (NVIDIA's example value) -- calibrated wheel_radius=0.0337 m (measured from this run's actual physics)
LECTURE: ground truth final pose      -> x=0.6293 y=0.3338 theta=98.68 deg
LECTURE: odometry (nominal radius)    -> x=0.6299 y=0.3273 theta=97.26 deg -- position error=0.0065 m
LECTURE: odometry (calibrated radius) -> x=0.5948 y=0.3575 theta=109.30 deg -- position error=0.0419 m
LECTURE: the calibrated radius made position error 6.4x WORSE, not better -- it overcorrected theta by ~12.3% ...
```

## Walking through it

**The setup: calibrate a documented constant against the real robot,
the same habit this course has applied to sensor conventions.**
`DifferentialController(wheel_radius=0.03, wheel_base=0.1125)` uses the
exact values from NVIDIA's own `jetbot_differential_move.py`. Commanding
`0.1 m/s` for 3 seconds should move the robot `0.300m` if that constant
is right. It actually moved `0.337m` — `12.3%` more. Scaling
`wheel_radius` up by that same `12.3%` (to `0.0337m`) makes the
*forward* prediction match reality by construction — that was the plan,
and Phase A alone would call it a success.

**Phase B's result contradicts the plan.** Driving forward, then
turning (`ω=1.0 rad/s`), then forward again, and dead-reckoning odometry
from the wheels' *actual* measured velocities (not the commanded ones —
this matters, see below) with both radii in parallel: the **nominal**
radius lands within `6.5mm` of the true final position; the
**calibrated** radius misses by `41.9mm` — six times worse. The
calibrated estimate's heading (`109.30°`) overshoots the true heading
(`98.68°`) by almost exactly the `12.3%` correction factor, while the
nominal estimate's heading (`97.26°`) tracks it closely.

**Why: the same constant appears in two different equations, and only
one of them was tested.** The inverse kinematic model this script uses
is
```
v = (v_left + v_right) / 2          # translational speed
ω = (v_right - v_left) / wheel_base # yaw rate
```
where `v_left = ω_left · r` and `v_right = ω_right · r` — `r`
(`wheel_radius`) scales *both* equations identically. Phase A only ever
exercised the `v` equation (`ω_left = ω_right`, so the `ω` equation was
always zero regardless of `r`). The `12.3%` discrepancy Phase A measured
therefore only characterizes what `r` does to straight-line motion —
it says nothing about whether that same `12.3%` correction is the right
one for the yaw term. Applying it there anyway is exactly why the
heading estimate overshot by roughly that same percentage: not a
coincidence, a direct consequence of one scalar doing two jobs.

**A harder question this lecture doesn't answer, on purpose: what
*causes* the 12.3%?** It's tempting to read "measured distance exceeds
commanded distance" as "the simulated wheel's true rolling radius is
bigger than 0.03m" — a pure-geometry explanation, and the one this
script's calibration step implicitly assumes. But an open-loop velocity
drive ramping up to a target can also produce a brief overshoot in
*achieved* velocity before settling, which would inflate measured
distance over a short run without the wheel's actual geometry being off
at all — and a transient like that has no reason to scale the *turning*
segment's wheel-velocity-asymmetry the same way it scaled the earlier
constant-velocity segment. This script can't distinguish those two
explanations from the data it collects, and that's the point: "my
corrected model matches reality better in the one test I ran" is a
weaker claim than "I found the right physical constant," and mistaking
one for the other is precisely how a calibration step makes things
worse instead of better.

**Odometry reads actual wheel velocity, not the commanded target —
deliberately.** `get_dof_velocities()`, not the `velocities` array
`apply_wheel_actions()` was just called with. A real robot's encoders
report what the wheels actually did, not what was asked of them; using
the commanded value here would make the dead-reckoning trivially
reconstruct the *controller's* intent rather than test anything about
the *physics*.

## Try it yourself

1. Add a rotation-only calibration phase (`v=0, ω=1.0 rad/s` for a fixed
   duration, measuring actual `Δtheta` via `world_pose()` before/after)
   and derive a **separate** correction factor for the `ω` equation
   instead of reusing the one from Phase A's straight-line run. Does a
   two-axis calibration beat both single-radius estimates?
2. Increase `CALIB_STEPS` from `180` to `600` (10s instead of 3s) and
   rerun Phase A alone. If the `12.3%` figure shrinks as the run gets
   longer, that's evidence for the transient/overshoot explanation,
   not the fixed-geometry one — a short run would still be dominated by
   the startup ramp, a long one would average it out.
3. Change `SEGMENTS`' turn angular speed from `1.0` to `2.5 rad/s` (a
   much sharper turn, more wheel slip) and compare how much `err_nominal`
   itself grows. Slip is the error source *no* radius calibration can
   fix — does the gap between "achievable with perfect calibration" and
   "what this run actually got" widen as the turn gets more aggressive?

## Next

[Lecture 19 — Module 2 capstone](lecture19.md): sensing, mapping,
planning, and now driving — put the last four lectures' pieces together
in one script.
