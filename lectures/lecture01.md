# Lecture 01 — Hello Simulation

**Code:** [`lecture01.py`](lecture01.py)

## The one thing this lecture teaches

Isaac Sim is not a library you `import` and start calling. It is an
application — Omniverse **Kit** — that your Python script *is running inside
of*, and that application has to boot before almost anything else works.
`SimulationApp` is the object that boots it.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture01.py
```

## What you'll see

A wall of `[ext: ...] startup` lines (Kit loading its extensions — normal,
takes several seconds), then:

```
LECTURE: starting -- about to construct SimulationApp (this is where Kit boots)
LECTURE: SimulationApp constructed -- omni.* and pxr are importable now
LECTURE: Kit version = 6.0.1
LECTURE: USD version = (0, 25, 11)
LECTURE: pumped 5 update() calls
LECTURE: closing
```

Notice what's *missing*: the script's very last `print` never appears. That's
not a bug — keep reading.

## Walking through it

**Import order is not a style preference.** `from isaacsim import
SimulationApp` has to be the first Isaac-Sim-flavored thing your script does.
`SimulationApp({"headless": True})` is what actually loads Kit's plugin
system — and `omni.*` / `pxr` are C++ plugins exposed to Python *by* that
system. Before the constructor runs, they are not partially-available or
slow to import — they do not exist as importable modules at all:

```pycon
>>> import omni.usd
ModuleNotFoundError: No module named 'omni.usd'
```

That's why the commented-out two lines near the top of `lecture01.py` are
positioned *before* the `SimulationApp` construction, not after: uncomment
them and you'll get exactly that traceback. This single ordering rule is
responsible for a large fraction of "why does my import fail" confusion
people hit in their first week with this API.

**`headless=True` is a real architectural choice, not a debug flag.**
Headless mode skips creating a window and a UI compositor thread. Every
lecture in this course runs headless, because it needs to work identically
whether you're at a desk with a monitor or SSH'd into a GPU box with none —
which, for anything you intend to automate (batch dataset generation, CI,
a training loop that resets a scene thousands of times), is the mode you
actually want. Flip the flag to `False` if you have a display and want to
watch a window open — it stays open until something closes it.

**`kit.update()` is the heartbeat.** Kit doesn't do anything on a background
thread while your script is sitting idle. Extensions finish initializing,
the renderer advances a frame, physics steps forward — all of it happens
*inside* a call to `update()`, and nowhere else. A script that never calls
it will sit there having constructed an app that never actually runs. Every
later lecture's "wait for something to happen" is a loop of these calls.

**`kit.close()` ends the process, not just the app object.** This is the
detail that trips people up once they start writing longer pipelines. Try
predicting the output of `lecture01.py` before running it — specifically,
does the very last `print()` statement execute? It doesn't. `close()` isn't
Python-level cleanup you could skip and still reach the rest of your script;
it tears down the process it's running in. Practically, this means: if a
task needs "import a robot, THEN configure its physics, THEN add cameras,"
that cannot be three functions called in sequence after one `SimulationApp`.
It has to be three separate script invocations from a shell wrapper, each
booting and closing its own `SimulationApp`. You'll see this pattern in every
real multi-stage Isaac Sim pipeline once you know to look for it.

## Try it yourself

1. Uncomment the two lines at the top of the script and re-run it. Read the
   traceback — is it the failure you expected?
2. Change `headless` to `False` (needs a display). What shows up, and does
   the script still reach `kit.close()` on its own, or do you have to close
   the window?
3. Move the final `print()` to *before* `kit.close()`. Confirm it prints now
   — you didn't misread the rule, the position is what matters.

## If you're on a different Isaac Sim version

Isaac Sim 5.0.0 and 6.0.1 both use `from isaacsim import SimulationApp` —
this lecture's code is identical on either. Versions older than 4.x used a
different import path, `from omni.isaac.kit import SimulationApp` — same
class, same constructor argument, just a different home before NVIDIA's 4.x
extension rename.

## Next

[Lecture 02 — Stage & USD Basics](lecture02.md): now that Kit is running,
what is the "scene" actually made of?
