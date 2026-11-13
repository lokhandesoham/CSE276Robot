"""Fit the systematic motion errors of a run from the EKF's own innovations (no ground truth).

    tools/.venv/bin/python tools/slam_fit_motion.py RUN.jsonl [--loops 2]

Re-runs the filter on the log for a grid of (turn scale, drive scale, drift) and keeps the one
with the most consistent measurements: the smallest mean normalized innovation squared
(NIS = nu^T S^-1 nu, ~2-3 for a consistent filter). Prints the values to start the next run with.
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'robot_control'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from slam_replay import replay  # noqa: E402


def mean_nis(statuses):
    vals = []
    for s in statuses:
        if 'Mahalanobis' in s[4]:
            vals.append(min(float(s[4].split('Mahalanobis ')[1].split(',')[0]), 30.0))
    return float(np.mean(vals)) if vals else float('inf')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--loops', type=int, default=0, help='use only the first N loops (0 = all)')
    a = ap.parse_args()
    records = [json.loads(x) for x in open(a.log)]
    if a.loops:
        cut, n = len(records), 0
        for i, r in enumerate(records):
            if r['type'] == 'loop':
                n += 1
                if n == a.loops:
                    cut = i + 1
                    break
        records = records[:cut]
    h = records[0]
    ts, ds, dr0 = h.get('turn_scale', 1.0), h.get('drive_scale', 1.0), h.get('drift', 0.0)
    rows = []
    for kt, kd, dr in itertools.product(np.arange(0.70, 1.21, 0.05), np.arange(0.84, 1.09, 0.04),
                                        np.radians(np.arange(-12, 12.1, 2.0))):
        slam, _, _, _, st = replay(records, None, None, None, kt, kd, dr0 + dr)
        rows.append((mean_nis(st), kt, kd, dr0 + dr, slam.recoveries))
    rows.sort()
    _, _, _, _, st = replay(records, None, None, None)
    print(f'as run: turn x{ts:.2f} drive x{ds:.2f} drift {math.degrees(dr0):+.1f} deg/m, '
          f'mean NIS {mean_nis(st):.2f}')
    for nis, kt, kd, dr, rc in rows[:5]:
        print(f'  NIS {nis:.2f}: robot turns x{kt:.2f}, drives x{kd:.2f} of the logged motion, '
              f'drift {math.degrees(dr):+.1f} deg/m, recoveries {rc}')
    _, kt, kd, dr, _ = rows[0]
    print(f'\nnext run: TURN_SCALE={ts * kt:.2f} DRIVE_SCALE={ds * kd:.2f} DRIFT={math.degrees(dr):.0f}')


if __name__ == '__main__':
    main()
