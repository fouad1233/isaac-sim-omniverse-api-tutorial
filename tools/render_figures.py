#!/usr/bin/env python3
"""Render lecture figures from the raw data lectures/*.py dump when run.

Why a separate script instead of matplotlib calls inside the lecture
scripts themselves: Isaac Sim's embedded Python (kit/python/bin/python3)
doesn't have matplotlib, and every other lecture in this course only
depends on numpy -- adding a plotting dependency to lecture11.py etc. just
to produce a doc image would be a bigger footprint than the doc image is
worth. Instead the lecture scripts dump the real numbers they already
computed (lectures/data_lectureNN.npz, gitignored -- regenerate by running
the lecture) and this script, run with an ordinary Python that has
matplotlib, turns those into the PNGs actually committed under
lectures/figures/.

Usage:
    python3 tools/render_figures.py          # renders every figure it has data for
    python3 tools/render_figures.py 11 16    # renders only lecture 11 and 16's figures

Run the lecture(s) you want fresh figures for first -- this script only
reads what's already on disk, it doesn't invoke Isaac Sim itself.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

LECTURES_DIR = Path(__file__).resolve().parent.parent / "lectures"
FIGURES_DIR = LECTURES_DIR / "figures"

# UNKNOWN, FREE, OCCUPIED -- matches the UNKNOWN/FREE/OCCUPIED = 0,1,2
# convention lectures 15/16/19 all use.
GRID_COLORS = ["#dcdcdc", "#ffffff", "#2b2b2b"]
GRID_CMAP = matplotlib.colors.ListedColormap(GRID_COLORS)


def _subsample(n: int, target: int) -> np.ndarray:
    """Evenly-strided indices, not random -- so re-running this script on
    the same .npz always produces the same figure."""
    stride = max(1, n // target)
    return np.arange(0, n, stride)


def render_lecture04(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    ref_size, flat_size = int(d["ref_size"]), int(d["flat_size"])

    fig, ax = plt.subplots(figsize=(5.5, 4.5))
    bars = ax.bar(
        ["referencing\n(points elsewhere)", "flattened\n(self-contained)"],
        [ref_size, flat_size],
        color=["#1f6feb", "#9a6700"],
    )
    ax.bar_label(bars, labels=[f"{ref_size} B", f"{flat_size} B"], padding=3)
    ax.set_ylabel("file size (bytes)")
    ax.set_title("Lecture 04 -- referencing vs. flattened .usda,\nreal measured file sizes")
    ax.margins(y=0.15)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture04_file_sizes.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def render_lecture05(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    steps = d["steps"]

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(steps, d["none_heights"], "o-", c="#2b2b2b", linewidth=2, markersize=5, label="no PhysicsScene (implicit)")
    ax.plot(steps, d["earth_heights"], "x--", c="#1f6feb", linewidth=1.5, markersize=7, label="PhysicsScene, g=9.81")
    ax.plot(steps, d["weak_heights"], "s:", c="#d1242f", linewidth=1.5, markersize=5, label="PhysicsScene, g=2.0")
    ax.set_xlabel("simulation step")
    ax.set_ylabel("box height, z (m)")
    ax.set_title("Lecture 05 -- falling box height over time\nsame start height 2.0 m, three gravity setups")
    ax.legend(loc="upper right")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture05_falling_box_height.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


PHASE_COLORS = {"playing": "#1f6feb", "paused": "#d1242f", "resumed": "#2da44e", "stopped": "#9a6700"}


def render_lecture06(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    heights, labels = d["heights"], d["labels"]
    steps = np.arange(len(heights))

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(steps, heights, "-", c="#2b2b2b", linewidth=1, zorder=2)
    seen = set()
    for phase in ("playing", "paused", "resumed", "stopped"):
        mask = labels == phase
        label = phase if phase not in seen else None
        seen.add(phase)
        ax.scatter(steps[mask], heights[mask], s=30, c=PHASE_COLORS[phase], label=label, zorder=3)
    # Shade each contiguous phase band so the play/pause/resume/stop
    # boundaries are visible at a glance, not just inferred from dot color.
    boundaries = [0] + [i for i in range(1, len(labels)) if labels[i] != labels[i - 1]] + [len(labels)]
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        ax.axvspan(start - 0.5, end - 0.5, color=PHASE_COLORS[labels[start]], alpha=0.08, zorder=1)
    ax.set_xlabel("update() call index")
    ax.set_ylabel("box height, z (m)")
    ax.set_title("Lecture 06 -- box height across play / pause / resume / stop")
    ax.legend(loc="center right")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture06_timeline_phases.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def render_lecture11(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    az_deg, dist_m = d["az_deg"], d["dist_m"]
    idx = _subsample(len(az_deg), 25_000)
    az, dist = az_deg[idx], dist_m[idx]
    x = dist * np.cos(np.radians(az))
    y = dist * np.sin(np.radians(az))

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(x, y, s=1, c="#1f6feb", alpha=0.5, linewidths=0)
    ax.scatter([0], [0], marker="^", s=120, c="#d1242f", zorder=5, label="lidar (sensor origin)")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m, East+)")
    ax.set_ylabel("y (m, North+)")
    ax.set_title(f"Lecture 11 -- 2D LiDAR scan, top-down\n{len(az_deg):,} beam returns ({len(idx):,} plotted)")
    ax.legend(loc="upper right")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture11_scan_topdown.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def render_lecture12(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    az_deg, el_deg, dist_m, ch = d["az_deg"], d["el_deg"], d["dist_m"], d["channel_id"]
    idx = _subsample(len(az_deg), 40_000)
    az, el, dist, channel = az_deg[idx], el_deg[idx], dist_m[idx], ch[idx]

    az_r, el_r = np.radians(az), np.radians(el)
    x = dist * np.cos(el_r) * np.cos(az_r)
    y = dist * np.cos(el_r) * np.sin(az_r)
    z = dist * np.sin(el_r)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(x, y, z, s=1.5, c=el, cmap="viridis", alpha=0.6, linewidths=0)
    ax.scatter([0], [0], [0], marker="^", s=150, c="#d1242f", zorder=5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m, sensor-relative)")
    ax.set_title(
        f"Lecture 12 -- 3D LiDAR scan, {len(np.unique(ch))} elevation rings\n"
        f"{len(az_deg):,} beam returns ({len(idx):,} plotted), color = elevation angle"
    )
    fig.colorbar(sc, ax=ax, shrink=0.6, label="elevation (deg)")
    ax.view_init(elev=28, azim=-60)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture12_scan_3d.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def _draw_grid(ax, grid, x_min, x_max, y_min, y_max):
    ax.imshow(
        grid, origin="lower", cmap=GRID_CMAP, vmin=0, vmax=2,
        extent=[x_min, x_max, y_min, y_max], interpolation="nearest",
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")


def render_lecture15(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    grid = d["grid"]
    n_free = int((grid == 1).sum())
    n_occ = int((grid == 2).sum())
    n_unk = int((grid == 0).sum())

    fig, ax = plt.subplots(figsize=(7, 6))
    _draw_grid(ax, grid, d["x_min"], d["x_max"], d["y_min"], d["y_max"])
    ax.set_title(
        "Lecture 15 -- occupancy grid from one lidar scan\n"
        f"{n_free} free (white) / {n_occ} occupied (black) / {n_unk} unknown (gray), "
        f"{float(d['res'])}m/cell"
    )
    fig.tight_layout()
    out = FIGURES_DIR / "lecture15_occupancy_grid.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def _grid_bounds(d):
    return float(d["x_min"]), float(d["x_max"]), float(d["y_min"]), float(d["y_max"]), float(d["res"])


def _cell_to_world(row, col, x_min, y_min, res):
    return x_min + (col + 0.5) * res, y_min + (row + 0.5) * res


def render_lecture16(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    grid = d["grid"]
    x_min, x_max, y_min, y_max, res = _grid_bounds(d)
    path = d["path"]
    start, goal = d["start"], d["goal"]

    path_x, path_y = _cell_to_world(path[:, 0], path[:, 1], x_min, y_min, res)
    start_x, start_y = _cell_to_world(start[0], start[1], x_min, y_min, res)
    goal_x, goal_y = _cell_to_world(goal[0], goal[1], x_min, y_min, res)

    fig, ax = plt.subplots(figsize=(7, 6))
    _draw_grid(ax, grid, x_min, x_max, y_min, y_max)
    ax.plot(path_x, path_y, "-", c="#d1242f", linewidth=2.5, label="A* path")
    ax.scatter([start_x], [start_y], marker="o", s=140, c="#1f6feb", zorder=5, label="start")
    ax.scatter([goal_x], [goal_y], marker="*", s=220, c="#2da44e", zorder=5, label="goal")
    ax.set_title(
        f"Lecture 16 -- A* path around the pillar\n"
        f"{len(path)} cells, straight line blocked, detour found instead"
    )
    ax.legend(loc="lower left", fontsize=9)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture16_astar_path.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def render_lecture19(data_path: Path) -> Path | None:
    if not data_path.exists():
        return None
    d = np.load(data_path)
    grid = d["grid"]
    x_min, x_max, y_min, y_max, res = _grid_bounds(d)
    path = d["path"]
    waypoints = d["waypoints"]
    traj_gt = d["traj_gt"]
    traj_odo = d["traj_odo"]

    path_x, path_y = _cell_to_world(path[:, 0], path[:, 1], x_min, y_min, res)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    _draw_grid(ax, grid, x_min, x_max, y_min, y_max)
    ax.plot(path_x, path_y, "--", c="#9a6700", linewidth=1.5, alpha=0.8, label="A* planned path")
    ax.scatter(waypoints[:, 0], waypoints[:, 1], marker="D", s=40, c="#9a6700", zorder=4, label="simplified waypoints")
    ax.plot(traj_gt[:, 0], traj_gt[:, 1], "-", c="#1f6feb", linewidth=2.2, label="driven (ground truth)")
    ax.plot(traj_odo[:, 0], traj_odo[:, 1], ":", c="#d1242f", linewidth=2.0, label="driven (wheel odometry belief)")
    ax.scatter([traj_gt[0, 0]], [traj_gt[0, 1]], marker="o", s=130, c="#1f6feb", zorder=5, label="start")
    ax.scatter([traj_gt[-1, 0]], [traj_gt[-1, 1]], marker="*", s=220, c="#2da44e", zorder=5, label="final pose")
    ax.set_title(
        "Lecture 19 -- capstone: scan-derived map, planned path, and\n"
        "actual driven trajectory vs. dead-reckoned belief"
    )
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    out = FIGURES_DIR / "lecture19_capstone_map_path_drive.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


RENDERERS = {
    4: render_lecture04,
    5: render_lecture05,
    6: render_lecture06,
    11: render_lecture11,
    12: render_lecture12,
    15: render_lecture15,
    16: render_lecture16,
    19: render_lecture19,
}


def main() -> None:
    requested = [int(a) for a in sys.argv[1:]] if len(sys.argv) > 1 else sorted(RENDERERS)
    FIGURES_DIR.mkdir(exist_ok=True)
    for n in requested:
        renderer = RENDERERS.get(n)
        if renderer is None:
            print(f"no renderer for lecture {n}, skipping")
            continue
        data_path = LECTURES_DIR / f"data_lecture{n:02d}.npz"
        out = renderer(data_path)
        if out is None:
            print(f"lecture {n}: no {data_path.name} found -- run the lecture first")
        else:
            print(f"lecture {n}: wrote {out}")


if __name__ == "__main__":
    main()
