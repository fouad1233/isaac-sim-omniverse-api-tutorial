"""Lecture 03 -- Transforms and the rotation-order convention.

Run it:
    <isaac-sim>/python.sh lectures/lecture03.py
"""

from isaacsim import SimulationApp

kit = SimulationApp({"headless": False})

import math  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

# Lecture 2 used the raw Usd.Stage.CreateNew()/CreateInMemory() API on
# purpose, to show a Stage as an object independent of any viewport. That
# independence is exactly why it's the wrong call here: this lecture runs
# with headless=False so you can watch a shape rotate, and a Stage built
# that way is NEVER shown by the GUI window -- the viewport only ever
# renders whatever omni.usd.get_context() holds. ctx.new_stage() creates
# the Stage IN the context to begin with, so anything defined on it below
# shows up immediately.
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# Step 1: translate / rotate / scale, the way you'll do it in every later
# lecture. XformCommonAPI is the friendly wrapper -- it hides the fact that
# under the hood a prim's transform is really a stack of separate "xformOp"
# entries (translate, then rotateXYZ, then scale, applied in THAT order),
# and just gives you one SetTranslate/SetRotate/SetScale call each.
thing = UsdGeom.Xform.Define(stage, "/World/Thing")
xf = UsdGeom.XformCommonAPI(thing)
xf.SetTranslate((1.0, 2.0, 0.5))
xf.SetRotate((0.0, 0.0, 90.0))  # degrees; this tuple is (rotateX, rotateY, rotateZ)
xf.SetScale((1.0, 1.0, 1.0))

# An Xform alone has no geometry -- nothing to actually see. This Cube is
# purely so the window shows a shape instead of an invisible transform
# node; everything the rest of this lecture proves is about /World/Thing's
# matrix, not about the cube itself.
box = UsdGeom.Cube.Define(stage, "/World/Thing/Box")
box.CreateSizeAttr(1.0)
box.CreateDisplayColorAttr([(0.9, 0.3, 0.1)])

for _ in range(10):
    kit.update()

world_matrix = thing.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
print("LECTURE: world matrix after translate=(1,2,0.5), rotateZ=90:")
for row in range(4):
    print(f"LECTURE:   {[round(world_matrix[row][c], 3) for c in range(4)]}")

# Step 2: the rotation-order convention -- proved, not just asserted.
#
# XformCommonAPI's rotate tuple is (rx, ry, rz) IN DEGREES, and it composes
# as R = Rz(rz) . Ry(ry) . Rx(rx) -- rotate about local X first, then Y,
# then Z. This is the single most common source of "my camera/light is
# pointing the wrong way" bugs, because it's easy to assume the tuple order
# IS the application order (rotate about X, that result about Y, that
# result about Z, each in the frame left behind by the previous step) --
# which gives a DIFFERENT answer than composing left-to-right as written.
#
# Rather than take that on faith, this builds R by hand in plain numpy and
# checks it against what USD itself computes for a real prim.


def rot_x(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


test_cases = [
    (0.0, -90.0, 0.0),   # a rotation you'll meet again in lecture 7 (camera aiming)
    (90.0, 0.0, 0.0),
    (30.0, 45.0, 60.0),  # all three axes at once, nothing conveniently zero
]

print("\nLECTURE: proving R = Rz . Ry . Rx by comparing against USD's own math")
for rx, ry, rz in test_cases:
    probe = UsdGeom.Xform.Define(stage, "/World/Probe")
    UsdGeom.XformCommonAPI(probe).SetRotate((rx, ry, rz))
    usd_matrix = probe.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # USD's Matrix4d is row-major and used as (row-vector) * matrix, i.e. the
    # rotation part sits transposed relative to the column-vector convention
    # rot_x/y/z above use -- pull it out and transpose to compare apples to
    # apples.
    usd_rot = np.array([[usd_matrix[r][c] for c in range(3)] for r in range(3)]).T

    hand_rot = rot_z(rz) @ rot_y(ry) @ rot_x(rx)

    # Apply both to local -Z, the "forward" direction every camera and every
    # UsdLux light in this course points along by default.
    local_forward = np.array([0.0, 0.0, -1.0])
    usd_forward = usd_rot @ local_forward
    hand_forward = hand_rot @ local_forward

    agree = np.allclose(usd_forward, hand_forward, atol=1e-6)
    print(
        f"LECTURE:   rotateXYZ={(rx, ry, rz)}  "
        f"USD forward={np.round(usd_forward, 3)}  "
        f"hand-computed Rz.Ry.Rx forward={np.round(hand_forward, 3)}  "
        f"match={agree}"
    )
    assert agree, "the stated convention just failed to reproduce USD's own answer"
    stage.RemovePrim(probe.GetPath())

print("\nLECTURE: all cases matched -- R = Rz(rz) . Ry(ry) . Rx(rx) is confirmed, not assumed")

# A quick render of /World/Thing/Box, the actual prim translate=(1,2,0.5),
# rotateZ=90 was applied to above -- so this lecture's .md has the real
# transformed box to show, not just a printed matrix.
import os  # noqa: E402

from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import UsdLux  # noqa: E402

UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(3000.0)
cam_geom = UsdGeom.Camera.Define(stage, "/World/PreviewCam")
# Same (+5,-5,+4) offset and (60,0,45) aim Lecture 02 used, just re-centered
# on /World/Thing's position instead of the origin -- the offset direction
# is what the rotation encodes, not the absolute position, so it still
# points straight at the box from here.
UsdGeom.XformCommonAPI(cam_geom).SetTranslate((6.0, -3.0, 4.5))
UsdGeom.XformCommonAPI(cam_geom).SetRotate((60.0, 0.0, 45.0))
preview_cam = Camera(prim_path="/World/PreviewCam", resolution=(320, 240))
preview_cam.initialize()

import omni.timeline  # noqa: E402

timeline = omni.timeline.get_timeline_interface()
timeline.play()
rgba = None
for _ in range(60):
    kit.update()
    rgba = preview_cam.get_rgba()
    if rgba is not None and rgba.size > 0:
        break
if rgba is None or rgba.size == 0:
    raise RuntimeError("preview camera never produced a frame in 60 steps")

from PIL import Image  # noqa: E402

preview_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output_lecture03_preview.png")
Image.fromarray(rgba, mode="RGBA").save(preview_path)
print(f"LECTURE: saved preview render to {preview_path}")
timeline.stop()
stage.RemovePrim("/World/PreviewCam")
stage.RemovePrim("/World/Sun")

# kit.update() runs as fast as it can (hundreds of calls/sec) whether or
# not a window is attached -- it does NOT pace itself to real time. Without
# this, the window would appear and `kit.close()` would tear it back down
# again well under a second later, too fast to actually look at the
# rotated box.
HOLD_SECONDS = 5.0
print(f"LECTURE: holding the window open for {HOLD_SECONDS:.0f}s -- look at it now")
t_end = time.time() + HOLD_SECONDS
while time.time() < t_end:
    kit.update()

kit.close()
