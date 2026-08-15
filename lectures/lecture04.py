"""Lecture 04 -- Composition: References vs Flattening.

Run it:
    <isaac-sim>/python.sh lectures/lecture04.py

Writes three files next to this script:
    output_lecture04_asset.usda      -- a standalone little "asset"
    output_lecture04_referenced.usda -- a scene that REFERENCES the asset
    output_lecture04_flattened.usda  -- the same scene, but EXPORTED (flattened)
"""

import os

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ASSET_PATH = os.path.join(HERE, "output_lecture04_asset.usda")
REFERENCED_PATH = os.path.join(HERE, "output_lecture04_referenced.usda")
FLATTENED_PATH = os.path.join(HERE, "output_lecture04_flattened.usda")
for p in (ASSET_PATH, REFERENCED_PATH, FLATTENED_PATH):
    if os.path.exists(p):
        os.remove(p)
moved_leftover = ASSET_PATH + ".moved"
if os.path.exists(moved_leftover):  # a previous run crashed before restoring it
    os.rename(moved_leftover, ASSET_PATH)
    os.remove(ASSET_PATH)

# Step 1: author a tiny standalone "asset" -- stand-in for a table, a robot,
# an environment, anything someone else built as its own .usd file.
asset_stage = Usd.Stage.CreateNew(ASSET_PATH)
UsdGeom.Xform.Define(asset_stage, "/Asset")
cube = UsdGeom.Cube.Define(asset_stage, "/Asset/Body")
cube.CreateSizeAttr(2.0)
asset_stage.SetDefaultPrim(asset_stage.GetPrimAtPath("/Asset"))
asset_stage.GetRootLayer().Save()
print(f"LECTURE: wrote standalone asset to {os.path.basename(ASSET_PATH)}")

# Step 2: bring it into a new scene by REFERENCE, not by rebuilding it.
# GetReferences().AddReference() with a bare filename (no directory) works
# because USD resolves a relative reference relative to the LAYER THAT
# CONTAINS IT, not your process's current working directory. This is the
# single most common "works when I run it from the repo root, breaks from
# anywhere else" bug in real pipelines -- it isn't actually broken, the
# reference was always relative to the wrong thing in the mental model.
referenced_stage = Usd.Stage.CreateNew(REFERENCED_PATH)
world = UsdGeom.Xform.Define(referenced_stage, "/World")
prop = referenced_stage.DefinePrim("/World/Prop", "Xform")
prop.GetReferences().AddReference(os.path.basename(ASSET_PATH))
referenced_stage.GetRootLayer().Save()
print(f"LECTURE: wrote referencing scene to {os.path.basename(REFERENCED_PATH)}")

# Prove the reference actually composed: /World/Prop/Body should now exist
# in the COMPOSED stage, even though nobody wrote "Body" into referenced_stage.
composed_path = "/World/Prop/Body"
found = referenced_stage.GetPrimAtPath(composed_path).IsValid()
print(f"LECTURE: {composed_path} valid via composition = {found}")

# Step 3: Export() -- ask for the fully composed result as ONE self-contained
# layer, with every referenced prim's data copied in directly.
referenced_stage.Export(FLATTENED_PATH)
print(f"LECTURE: exported flattened copy to {os.path.basename(FLATTENED_PATH)}")

print("\nLECTURE: raw text of the REFERENCING file (small, points elsewhere):")
with open(REFERENCED_PATH) as f:
    print("\n".join(f"LECTURE:   {line}" for line in f.read().splitlines()))

print("\nLECTURE: raw text of the FLATTENED file (bigger, self-contained):")
with open(FLATTENED_PATH) as f:
    print("\n".join(f"LECTURE:   {line}" for line in f.read().splitlines()))

ref_size = os.path.getsize(REFERENCED_PATH)
flat_size = os.path.getsize(FLATTENED_PATH)
print(f"\nLECTURE: file size -- referencing={ref_size}B  flattened={flat_size}B")

# Step 4: the part that actually matters in practice -- what happens if the
# original asset file goes away? Move it aside, then open EACH scene fresh
# (a brand new Stage.Open, nothing cached) and see which one still works.
moved_path = ASSET_PATH + ".moved"
os.rename(ASSET_PATH, moved_path)
print(f"\nLECTURE: moved {os.path.basename(ASSET_PATH)} out of the way")

fresh_referenced = Usd.Stage.Open(REFERENCED_PATH)
ref_prim = fresh_referenced.GetPrimAtPath(composed_path)
print(
    f"LECTURE: SAME PROCESS, referencing scene, asset missing -> "
    f"{composed_path} valid={ref_prim.IsValid()}"
)
print(
    "LECTURE:   (if that says True, don't trust it yet -- read the .md before "
    "concluding the reference survived losing its file)"
)

# The check above is not trustworthy, and that is itself the lesson: Sdf's
# layer registry is PROCESS-WIDE and keyed by resolved path. The asset layer
# got loaded into that registry back when asset_stage was created, earlier in
# THIS SAME process, and moving the file on disk doesn't evict it. A "fresh"
# Usd.Stage.Open() here is fresh at the stage level, not at the layer-cache
# level. To find out what really happens when the file is gone, ask a
# genuinely new process that has never touched these layers -- which, in
# practice, is exactly what "close Isaac Sim, come back tomorrow, someone
# deleted the asset" looks like.
import subprocess  # noqa: E402
import sys  # noqa: E402
import tempfile  # noqa: E402

python_sh = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "..", "..", "..", "python.sh"))
# NOT an f-string: the "{{...}}" pairs need to survive down to a literal
# single brace AFTER .format() runs below, for the dict literal Isaac Sim
# needs. Mixing an f-string with a later .format() call on the same text is
# its own small trap -- f-strings evaluate "{{" to "{" immediately, so by
# the time .format() sees the result there is no doubling left to protect
# the dict literal, and .format() tries to treat it as a field name instead.
checker_src = '''
from isaacsim import SimulationApp
kit = SimulationApp({{"headless": True}})
from pxr import Usd
stage = Usd.Stage.Open({path!r})
prim = stage.GetPrimAtPath({composed_path!r})
print("LECTURE_SUBPROCESS_RESULT", {path!r}, prim.IsValid())
kit.close()
'''
with tempfile.NamedTemporaryFile("w", suffix="_checker.py", delete=False) as f:
    checker_path = f.name

# Only the referencing scene, on purpose -- this spawns a second Isaac Sim
# boot (~a minute) to get a real answer, and the referencing scene is the
# one whose answer actually changes once you leave the same-process cache
# behind. The flattened scene's fresh-process check is exercise 1 below;
# running it yourself, the same way, is more convincing than reading it here.
with open(checker_path, "w") as f:
    f.write(checker_src.format(path=REFERENCED_PATH, composed_path=composed_path))
result = subprocess.run([python_sh, checker_path], capture_output=True, text=True)
line = next((ln for ln in result.stdout.splitlines() if "LECTURE_SUBPROCESS_RESULT" in ln), None)
valid = line.split()[-1] if line else "UNKNOWN (subprocess failed, see stderr below)"
print(f"LECTURE: FRESH PROCESS,   referencing scene, asset missing -> {composed_path} valid={valid}")
if line is None:
    print(result.stderr[-800:])
os.remove(checker_path)

os.rename(moved_path, ASSET_PATH)  # restore, so a re-run starts from the same state
print(f"\nLECTURE: restored {os.path.basename(ASSET_PATH)}")

kit.close()
