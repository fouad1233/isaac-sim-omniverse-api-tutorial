# Lecture 15 — Mapping (occupancy grid)

**Code:** [`lecture15.py`](lecture15.py)

## The one thing this lecture teaches

Lecture 11 turned one lidar scan into eight numbers — the nearest verified
distance in eight compass directions. An occupancy grid is the same
computation at full resolution: bin *every* azimuth, keep the nearest
return in each bin, then for every cell in a 2D grid compare its own
distance from the sensor to that direction's measured range. Nearer means
the beam passed through empty space to get further out; at-range means
that's what stopped it; farther means nothing is known, because whatever's
there is behind a wall the beam never reached. No per-point ray marching,
no new geometry, no new sensor code — the same `(azimuth, distance)` pairs
Lecture 11 already trusted, read into a grid instead of eight printouts.

## Run it

```bash
<your-isaac-sim-install>/python.sh lectures/lecture15.py
```

## What you'll see

```
LECTURE: captured 15152189 beam returns, azimuth range [-180.0, 180.0] deg, distance range [1.90, 4.87] m
LECTURE: 720 azimuth bins (0.50deg each), 0 received no return
LECTURE: grid is 35x45 cells at 0.2m/cell -- 759 free, 116 occupied, 700 unknown
LECTURE:   East (0deg)      nearest occupied range = 2.89 m
LECTURE:   North (90deg)    nearest occupied range = 2.89 m
LECTURE:   West (180deg)    nearest occupied range = 1.90 m
LECTURE:   South (-90deg)   nearest occupied range = 3.89 m

LECTURE: occupancy grid ('#'=occupied '.'=free ' '=unknown), x:[-3.0,4.0] y:[-5.0,4.0] at 0.2m/cell:
LECTURE:      #########################
LECTURE:      #.......................#
                        ...
LECTURE:      #.......................#
LECTURE:      #########################
```

(Full grid: 35 printed rows, a clean rectangle of `#` with `.` filling the
interior and blank margin around it — see the actual run for all of it.)

## Walking through it

**The compass numbers are identical to Lecture 11's, on purpose.** Same
sealed room, same lidar config, same `elementsCoordsType == SPHERICAL`
field reading, same sensor position — `2.89m` East, `2.89m` North,
`1.90m` West, `3.89m` South, matching Lecture 11's printed values exactly.
That repetition is the point: this lecture isn't re-deriving lidar
correctness, it's reusing an already-verified data source and building
something new — a grid — on top of it.

**Binning by azimuth and keeping the minimum is what "a wall can't be seen
through" means in code.** `np.minimum.at(bin_range, bin_idx, dist_m)`
collapses however many of the ~15 million returns land in each 0.5°
bin down to one number: the closest. Every bin in this run got at least
one return (`0 received no return`) — expected, since the room from
Lecture 11 is fully sealed and 200 ticks was already confirmed there to
cover a full rotation.

**The grid comparison is entirely vectorized, and the geometry it recovers
is exact, not approximate.** Every cell's own `(r, theta)` relative to the
sensor at the origin gets compared, all at once, to that direction's
binned range — no loop over 15 million points, no per-ray marching.
Checking the printed grid's actual occupied rectangle against the room's
real wall positions: `25` columns wide by `35` rows tall at `0.2m/cell` is
exactly `5.0m x 7.0m` — and the room's true span is East `x=3.0` to West
`x=-2.0` (`5.0m`) by North `y=3.0` to South `y=-4.0` (`7.0m`). Exact match,
not close-enough.

**"Unknown" is a real, distinct third state, not a fallback for "free."**
`700` of the grid's `1575` cells are unknown — everything outside the
sealed room, which this single static scan never had any way to observe.
Marking those cells `FREE` by default (a common shortcut) would silently
claim knowledge the sensor never provided. An occupancy grid's whole value
for navigation is knowing the difference between "measured clear" and
"never measured" — driving through the first is fine, driving through the
second is a guess.

**This is one scan, from one fixed position — a snapshot, not yet a
map that accumulates.** A real occupancy grid update rule (log-odds
Bayesian fusion, most commonly) merges many scans from many sensor poses
over time, so a single bad or occluded reading doesn't overwrite
previously-confirmed cells and unknown regions shrink as the sensor moves
through them. Building the grid from exactly one scan skips that entirely,
on purpose: it isolates "how does one scan become a grid" from "how do
many scans become a consistent one," which is a different problem with
its own failure modes.

## Try it yourself

1. Drop `N_BINS` from `720` to `36` (10° bins instead of 0.5°) and rerun.
   Does the printed rectangle's corners get visibly rounder, and does the
   `East/North/West/South` verification still land on the same four
   numbers?
2. Change `RES` from `0.2` to `0.05` and rerun. The occupied rectangle
   should still measure `5.0m x 7.0m` in real units — does it, even though
   the cell counts are now completely different?
3. This scan's sensor sits at world `(0, 0)`, not the room's true center
   `(0.5, -0.5)` — that offset is exactly why the four verified ranges
   aren't the room's "ideal" `3, 3, 4, 2`. Move the lidar's `translations`
   to `(0.5, -0.5, 1.0)` and rerun. Do the four compass ranges land closer
   to those round numbers now?

## Next

[Lecture 16 — Path planning](lecture16.md): now that a scan is a grid,
finding a route through it from one cell to another.
