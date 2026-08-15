"""Lecture 01 -- Hello Simulation: the SimulationApp lifecycle.

Run it:
    <isaac-sim>/python.sh lectures/lecture01.py

Expect several seconds of Kit extension-loading spam, then a handful of
lines starting with "LECTURE:". Read lecture01.md for what each one means
and why the script is structured in this exact order.
"""

# Step 0: SimulationApp must be created before ANYTHING else Isaac-Sim-flavored
# is imported. Uncomment the two lines below and run this file again -- it
# fails with ModuleNotFoundError, not a warning. Kit's plugin system, which is
# what makes `omni.*` and `pxr` importable at all, is loaded inside
# SimulationApp's constructor below. There is nothing to import yet, before
# that call runs.
#
#   import omni.usd
#   print("this cannot run yet")

print("LECTURE: starting -- about to construct SimulationApp (this is where Kit boots)")

from isaacsim import SimulationApp

# headless=True: no window, no GPU-composited UI thread. This is the mode you
# want for anything scripted -- SSH sessions, CI, batch rendering. Every
# lecture in this course runs headless so it works the same way on any
# machine. Flip it to False if you have a display attached and want to watch
# the window appear (it will sit there until you close it).
kit = SimulationApp({"headless": True})

print("LECTURE: SimulationApp constructed -- omni.* and pxr are importable now")

# Only after the SimulationApp() call above do these imports work.
import omni.kit.app  # noqa: E402
from pxr import Usd  # noqa: E402

app = omni.kit.app.get_app()
print(f"LECTURE: Kit version = {app.get_app_version()}")
print(f"LECTURE: USD version = {Usd.GetVersion()}")

# Step 1: pump the app. Kit is event-driven -- extensions finish loading, the
# renderer produces a frame, a physics step advances, all of it inside
# update(). Nothing happens on its own between update() calls, which is why
# every later lecture is built around a loop of them.
for _ in range(5):
    kit.update()
print("LECTURE: pumped 5 update() calls")

# Step 2: close it down. This is not optional cleanup you can skip because
# the script is about to exit anyway -- close() is what flushes Kit's
# extensions and the renderer cleanly. Skipping it (falling off the end of
# the script, or Ctrl-C) is how you get GPU-memory-still-held warnings on
# your next run.
print("LECTURE: closing")
kit.close()

# This next line is reachable Python -- but it will never print. close() ends
# the process it runs in; nothing after it executes, ever. This is why every
# multi-step Isaac Sim pipeline you'll see in the wild (import an asset, then
# configure its physics, then add cameras...) is chained as SEPARATE process
# invocations from a shell script, not as sequential function calls after one
# SimulationApp in a single Python file.
print("LECTURE: you will never see this line printed")
