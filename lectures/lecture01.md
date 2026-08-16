# Lecture 01 — Hello Simulation

**Code:** [`lecture01.py`](lecture01.py)

## The one thing this lecture teaches

Isaac Sim is not a library you `import` and start calling. It is an
application — Omniverse **Kit** — that your Python script *is running inside
of*, and that application has to boot before almost anything else works.
`SimulationApp` is the object that boots it.

## Before you run anything: fix your editor

If you're using VS Code, do this now — before lecture 1, not after you've
already hit it on lecture 6 and gone looking for a bug that isn't there.

**Symptom:** you Ctrl-click (or F12 / "Go to Definition") on `SimulationApp`,
`Camera`, `Lidar`, anything from `isaacsim.*` or `omni.*` — and nothing
happens. No error, no red squiggle telling you the import is unresolved,
just silence. Pylance genuinely cannot find these libraries, and it will not
tell you that's what's wrong.

**Why:** Isaac Sim doesn't `pip install` `isaacsim`/`omni`. Each of Kit's
~600 extensions ships its own fragment of these two packages under
`exts/<dotted.ext.name>/isaacsim/...` (or `.../omni/...`), and the extension
manager stitches the fragments together *at runtime*, based on which
extensions happen to be enabled for the app you launched. That stitching
happens through Kit's own loader, not a static `PYTHONPATH` — so there is no
list of "here's where everything is" for a static analyzer like Pylance to
read. It isn't a settings bug you're missing a checkbox for; the information
literally doesn't exist anywhere until the app is running.

There's a second trap hiding under the first one. Isaac Sim's own
`python_packages/isaacsim/__init__.py` is a real bootstrap module (it's what
decides which extensions to enable and wires up their paths when you use
`python.sh`) — and a real `__init__.py` file, by Python's own import rules,
**blocks** namespace-package merging for every other `isaacsim` fragment on
the path. Point Pylance at that directory thinking it'll help, and you get
the opposite: it commits to that one bootstrap file as the *entire*
`isaacsim` package and `isaacsim.sensors`, `isaacsim.core`, all of it,
silently stop resolving. (Found by bisection: adding that one directory to
an otherwise-working config was enough to break every `isaacsim.*` submodule
import, verified with `pyright` directly.)

**Fix:** this repo ships a generator that scans your actual install and
writes the exact, verified path list Pylance needs — run it once, from the
repo root, pointing at your own Isaac Sim install:

```bash
python3 tools/setup_vscode.py /path/to/your/isaacsim-install
```

Then reload the window (`Ctrl+Shift+P` → `Developer: Reload Window`). It
writes `.vscode/settings.json` with `python.defaultInterpreterPath` (Kit's
own Python, so hover/autocomplete match what actually runs) and
`python.analysis.extraPaths` (every `isaacsim`/`omni` fragment, minus the
bootstrap directory that breaks the merge). That file isn't checked into
this repo — the paths are absolute and specific to *your* machine, so it's
gitignored; you're expected to generate your own.

This was verified the same way every claim in this course is verified: not
by reasoning about how namespace packages are supposed to work, but by
running `pyright` (the engine Pylance is built on) against every lecture
script with the generated config and confirming zero unresolved imports —
then deliberately re-adding the bootstrap directory and watching the same
imports break, to isolate the actual cause rather than guess at it.

**A third trap, one symbol wide, hiding under the second one.** Excluding
`python_packages/` fixes `isaacsim.sensors`, `isaacsim.core`, every
dotted submodule — but every single lecture in this course starts with
`from isaacsim import SimulationApp`, the bare top-level import, and that
one still failed after the fix above: "SimulationApp" is unknown import
symbol, while `Camera`, `Lidar`, everything else resolved fine right next
to it. The cause is specific rather than structural: the excluded
bootstrap file doesn't expose `SimulationApp` with a static `from .x
import y` a type checker can trace to the real class — it does `AppFramework,
SimulationApp = expose_api()`, a runtime function call, so even a checker
that *could* see that file has nothing to statically follow. The real
class lives in `isaacsim.simulation_app.simulation_app.SimulationApp`,
already reachable through the submodule import — just not through the
top-level one every lecture actually uses.

The fix is a one-symbol type stub, not another entry in `extraPaths`:
`tools/setup_vscode.py` now also writes `.vscode/typings/isaacsim/__init__.pyi`
containing exactly
`from isaacsim.simulation_app import AppFramework as AppFramework, SimulationApp as SimulationApp`,
and points `python.analysis.stubPath` at that directory. A `.pyi` stub
resolved through `stubPath` is a separate lookup from the `extraPaths`
namespace-package merge — it can claim the one symbol `isaacsim` needs at
the top level without reintroducing the "regular `__init__.py` blocks the
merge" problem the real bootstrap file causes. Verified the same way as
the trap above: `pyright` against all 19 lecture scripts reports the
`SimulationApp` import as `type[SimulationApp]` (not `Unknown`) in every
one, and removing just the stub reproduces "unknown import symbol" in
all 19, with the other 103 pre-existing diagnostics (incomplete `pxr`/
`numpy` type stubs, unrelated to this fix) identical either way.

If you're not on VS Code, the underlying fact still matters: any tool that
does static import resolution (pyright standalone, mypy, an IDE's "jump to
source") needs this same fragment list, built the same way. Read
`tools/setup_vscode.py` — the scanning logic has nothing VS-Code-specific in
it.

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
