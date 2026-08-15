"""Lecture 02 -- Stage & USD Basics.

Run it:
    <isaac-sim>/python.sh lectures/lecture02.py

Writes output_lecture02.usda next to this script. Open it in a text editor
afterwards -- that's half the point of this lecture.
"""

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import os  # noqa: E402

from pxr import Usd, UsdGeom  # noqa: E402

OUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output_lecture02.usda")
if os.path.exists(OUT_PATH):
    os.remove(OUT_PATH)

# A Stage is what you actually program against. Right now it's backed by
# exactly one Layer -- the file at OUT_PATH. Stage and Layer look like the
# same thing until lecture 4, where composing several layers together is
# the entire point, so it's worth knowing from the start that they are two
# different objects that happen to be 1:1 here.
stage = Usd.Stage.CreateNew(OUT_PATH)
print(f"LECTURE: stage created, backed by {OUT_PATH}")

# A Prim is a named node in the stage's hierarchy -- think "a folder in a
# filesystem tree." DefinePrim(path, typeName) creates one. "Xform" is USD's
# transform-holding grouping type; most scenes' "/World" root is one.
world = stage.DefinePrim("/World", "Xform")
print(f"LECTURE: defined prim {world.GetPath()}  type={world.GetTypeName()}")

# UsdGeom.Cube.Define() is the typed-schema convenience form of the same
# call -- it does DefinePrim(path, "Cube") AND hands back a UsdGeom.Cube
# wrapper with cube-specific methods (CreateSizeAttr, etc.) instead of a
# bare untyped prim you'd have to poke attributes onto by hand.
box = UsdGeom.Cube.Define(stage, "/World/Box")
box.CreateSizeAttr(2.0)
print(f"LECTURE: defined prim {box.GetPath()}  type={box.GetPrim().GetTypeName()}")

# An Attribute is a named, typed value living ON a prim -- "size" above is
# one. GetAttributes() lists every attribute the prim's schema declares,
# whether or not a value was ever set for it -- IsAuthored() tells you which.
print("LECTURE: authored attributes on /World/Box:")
for attr in box.GetPrim().GetAttributes():
    if attr.IsAuthored():
        print(f"LECTURE:   {attr.GetName()} = {attr.Get()}")

# Traverse() walks every prim in the stage, depth-first. This is how tools
# (and your own scripts) discover "what's actually in this scene" without
# already knowing the paths in advance.
print("LECTURE: traversing the whole stage:")
for prim in stage.Traverse():
    print(f"LECTURE:   {prim.GetPath()}  ({prim.GetTypeName()})")

# Save() writes the in-memory edits back to OUT_PATH as human-readable .usda
# text. Go look at the file after this script exits -- USD's text format is
# not a mystery binary blob, it's meant to be read.
stage.GetRootLayer().Save()
print(f"LECTURE: saved to {OUT_PATH}")

with open(OUT_PATH) as f:
    saved_text = f.read()
print("LECTURE: first 15 lines of the saved file:")
for line in saved_text.splitlines()[:15]:
    print(f"LECTURE:   {line}")

kit.close()
