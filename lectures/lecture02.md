# Lecture 02 — Stage & USD Basics

**Code:** [`lecture02.py`](lecture02.py)

## The one thing this lecture teaches

Everything you'll ever build in Isaac Sim — a robot, a room, a physics
scene, a camera — is a node in a **USD (Universal Scene Description)**
tree. Isaac Sim's Python API is, underneath the robotics-flavored parts,
a fairly thin layer over `pxr`, Pixar's open-source USD library. Learn
`pxr`'s vocabulary once and it stops feeling like Isaac-Sim-specific magic.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture02.py
```

It writes `output_lecture02.usda` next to the script. Open that file in any
text editor after running — that's not optional, it's the point.

## What you'll see

```
LECTURE: stage created, backed by .../lectures/output_lecture02.usda
LECTURE: defined prim /World  type=Xform
LECTURE: defined prim /World/Box  type=Cube
LECTURE: authored attributes on /World/Box:
LECTURE:   size = 2.0
LECTURE: traversing the whole stage:
LECTURE:   /World  (Xform)
LECTURE:   /World/Box  (Cube)
LECTURE: saved to .../lectures/output_lecture02.usda
LECTURE: first 15 lines of the saved file:
LECTURE:   #usda 1.0
LECTURE:
LECTURE:   def Xform "World"
LECTURE:   {
LECTURE:       def Cube "Box"
LECTURE:       {
LECTURE:           double size = 2
LECTURE:       }
LECTURE:   }
LECTURE:
LECTURE: opened .../lectures/output_lecture02.usda in the GUI viewport
LECTURE: holding the window open for 5s -- look at it now
```

If you're running with a real display attached, a window opens and a gray
box sits in it for five seconds before the script exits. If you don't see
that, read the next section before assuming something's broken.

## Walking through it

**Stage vs. Layer.** A `Stage` is the object you actually call methods on —
it's the *composed, in-memory result*. A `Layer` is a source of edits (in
the simplest case, a `.usda`/`.usd` file on disk). `Usd.Stage.CreateNew(path)`
creates both at once and points the stage at a single root layer, so right
now they look interchangeable. They stop being interchangeable the moment
you reference one stage's content into another — which is the entirety of
[lecture 4](lecture04.md) — so it's worth having the two names straight
before that point, not during it.

**A Prim is a node, not a "thing."** `/World` and `/World/Box` are both
**prims** — the path is literally a filesystem-style path, and prims nest
the same way directories do. `DefinePrim(path, typeName)` is the low-level
constructor: give it a path and a schema name (`"Xform"`, `"Cube"`, ...)
and it creates a prim of that type at that path, creating parent prims
implicitly if they don't exist yet.

**Typed schemas give you a nicer API for the same thing.**
`UsdGeom.Cube.Define(stage, path)` does exactly what `DefinePrim(path,
"Cube")` does, but hands back a `UsdGeom.Cube` *wrapper object* with
schema-aware methods like `CreateSizeAttr()` instead of leaving you to
poke at raw attributes by name and type. Every `UsdGeom`/`UsdPhysics`/
`UsdLux` class you'll meet in later lectures follows this same shape:
`Xxx.Define(stage, path)` → a typed wrapper around a plain prim.

**An Attribute is a named, typed value on a prim.** `size` on a `Cube` is
one. The schema (`Cube`) *declares* which attributes exist and their
types and defaults; `GetAttributes()` returns all of them, but most report
`IsAuthored() == False` until you actually set a value — the schema's
default is used until then, and nothing is written to the file for an
attribute you never touched. This is why the saved `.usda` below only shows
`size`, even though a `Cube` prim has other attributes (`extent`,
`purpose`, `visibility`, ...) available by schema.

**`Traverse()` is how you discover a scene, not just build one.** Given an
arbitrary stage — someone else's asset, a scene you loaded from disk — you
almost never already know its prim paths. Walking `stage.Traverse()` and
checking `prim.GetTypeName()` / `prim.IsA(SomeSchema)` is the standard way
to find "give me every Cube in this scene" or "does this stage have a
physics scene at all" (a question [lecture 5](lecture05.md) answers the
hard way).

**The saved file is not a black box.** `.usda` is USD's *text* format —
human-readable, diffable, greppable, exactly like the 15 lines printed
above. (`.usd`/`.usdc` are the binary-encoded equivalent — same data
model, faster to load, not readable in a text editor. Isaac Sim happily
reads and writes either; this course sticks to `.usda` specifically so you
can open the output and look.) The nesting in the file mirrors the prim
hierarchy exactly: `Cube "Box"` sits inside `Xform "World"` because
`/World/Box` sits under `/World`.

**A `Stage` you build this way is invisible in the GUI window until you say
otherwise, and that's worth understanding rather than working around
blindly.** `Usd.Stage.CreateNew(path)` gives you a real, fully functional
Stage object — but it exists only in this Python process. The GUI window
(`headless=False`) renders whatever `omni.usd.get_context()` holds, and
that is a *separate* stage that Kit itself owns, empty by default apart
from its own default cameras and render settings. Querying it right after
`CreateNew()` finds no `/World`, no `/World/Box` — nothing this script
built. That's not a bug in USD; a `Usd.Stage` was never meant to assume
there's a viewport at all (headless server rendering is the far more common
real-world case). The fix is one explicit line: `omni.usd.get_context().
open_stage(OUT_PATH)`, handing the just-saved file to the object that
actually drives the window. Skipping it is exactly what produces the
symptom "I ran it with `headless=False` and the window is just empty."

**And even with that fixed, the window would close before you could look at
it.** `kit.update()` doesn't pace itself to real time — it ran at roughly
250 calls/second in testing, window or no window. Nothing between defining
the box and `kit.close()` normally takes more than a few milliseconds, so
without an explicit real-time wait, the window would appear and be torn
back down again well under a second later. The `while time.time() < t_end`
loop at the end is what actually holds it open for five real seconds — a
frame-count loop like Lecture 06 uses elsewhere would not have done this,
because frame count and wall-clock time are not the same thing once a
window's involved.

## Try it yourself

1. Add a second prim, `UsdGeom.Sphere.Define(stage, "/World/Ball")`, before
   `stage.GetRootLayer().Save()`. Re-run and check it shows up both in the
   traversal output and in the saved file.
2. Print `box.GetPrim().GetAttributes()` *without* the `IsAuthored()`
   filter. How many attributes does a bare `Cube` actually have available?
3. Open `output_lecture02.usda` and hand-edit `size` to a different number,
   save the file, then load it back with `Usd.Stage.Open(OUT_PATH)` in a
   throwaway script and print the value. USD doesn't care whether an edit
   came from your script or a text editor.

## Next

[Lecture 03 — Transforms](lecture03.md): `/World/Box` exists, but it has no
position yet. Where a prim *is* turns out to have a rotation-order gotcha
worth knowing before you hit it in a camera or a light.
