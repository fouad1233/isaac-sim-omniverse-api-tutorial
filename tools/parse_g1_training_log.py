#!/usr/bin/env python3
"""Parse an Isaac Lab rsl_rl train.py text log into lectures/data_lecture21.npz.

Why a separate step from the training run itself: `train.py` runs under
`isaaclab.sh -p` (Isaac Sim 5.0.0 / Python 3.11, no matplotlib), and its raw
log is many MB of console text, not something this repo commits. Point this
at the log `train.py` already wrote under
`<isaaclab-install>/logs/rsl_rl/<experiment_name>/<timestamp>/`, or at a
copy of it, and it extracts one row per "Learning iteration N/M" block into
the same data_lectureNN.npz convention tools/render_figures.py already reads
for every other lecture.

Usage:
    python3 tools/parse_g1_training_log.py <path-to-train-log.txt>
"""

import re
import sys
from pathlib import Path

import numpy as np

LECTURES_DIR = Path(__file__).resolve().parent.parent / "lectures"

ITER_RE = re.compile(r"Learning iteration (\d+)/(\d+)")
REWARD_RE = re.compile(r"Mean reward:\s*(-?[\d.]+)")
LIN_VEL_RE = re.compile(r"Episode_Reward/track_lin_vel_xy_exp:\s*(-?[\d.]+)")
ANG_VEL_RE = re.compile(r"Episode_Reward/track_ang_vel_z_exp:\s*(-?[\d.]+)")
BASE_CONTACT_RE = re.compile(r"Episode_Termination/base_contact:\s*(-?[\d.]+)")
EP_LEN_RE = re.compile(r"Mean episode length:\s*(-?[\d.]+)")


def parse(text: str) -> dict:
    # Split into one chunk per "Learning iteration N/M" block so each metric
    # regex only searches within its own iteration's text -- avoids pairing
    # iteration N's header with iteration N+1's reward if a field is ever
    # missing from one block.
    blocks = re.split(r"(?=Learning iteration \d+/\d+)", text)

    iterations, rewards, lin_vel, ang_vel, base_contact, ep_len = [], [], [], [], [], []
    for block in blocks:
        m_iter = ITER_RE.search(block)
        m_reward = REWARD_RE.search(block)
        if not (m_iter and m_reward):
            continue
        iterations.append(int(m_iter.group(1)))
        rewards.append(float(m_reward.group(1)))
        for pat, out in ((LIN_VEL_RE, lin_vel), (ANG_VEL_RE, ang_vel),
                         (BASE_CONTACT_RE, base_contact), (EP_LEN_RE, ep_len)):
            m = pat.search(block)
            out.append(float(m.group(1)) if m else np.nan)

    return {
        "iterations": np.array(iterations),
        "rewards": np.array(rewards),
        "lin_vel": np.array(lin_vel),
        "ang_vel": np.array(ang_vel),
        "base_contact": np.array(base_contact),
        "ep_len": np.array(ep_len),
    }


def main() -> None:
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    log_path = Path(sys.argv[1])
    data = parse(log_path.read_text())

    out_path = LECTURES_DIR / "data_lecture21.npz"
    np.savez(out_path, **data)

    its, r, bc, el = data["iterations"], data["rewards"], data["base_contact"], data["ep_len"]
    print(f"parsed {len(its)} iterations, range [{its.min()}, {its.max()}]")
    print(f"first reward={r[0]:.2f}, last reward={r[-1]:.2f}")
    print(f"first base_contact={bc[0]:.4f}, last base_contact={bc[-1]:.4f}")
    print(f"first ep_len={el[0]:.2f}, last ep_len={el[-1]:.2f}")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
