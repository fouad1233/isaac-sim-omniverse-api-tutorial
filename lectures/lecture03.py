"""Lecture 03 -- Transforms and the rotation-order convention.

Run it:
    <isaac-sim>/python.sh lectures/lecture03.py
"""

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import math  # noqa: E402

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

stage = Usd.Stage.CreateInMemory()
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

kit.close()
