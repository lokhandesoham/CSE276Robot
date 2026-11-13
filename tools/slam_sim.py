"""Simulate the HW3 run (no robot): closed-loop stop-and-go EKF-SLAM in the 8 ft arena.

Writes a .jsonl log in the same format as the robot's slam_follower, plus the true
trajectory, so tools/slam_replay.py can be tested end to end.

    tools/.venv/bin/python tools/slam_sim.py square --loops 3 --out runs/sim_square.jsonl
    tools/.venv/bin/python tools/slam_sim.py octagon --loops 3 --out runs/sim_octagon.jsonl
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'robot_control'))
from robot_control.ekf_slam import EkfSlam, fuse_look, load_path, next_motion, wrap_angle  # noqa: E402

FACING = {0: 90, 1: 90, 2: 180, 3: 180, 4: -90, 5: -90, 6: 0, 7: 0}  # planned layout
START = {'square': (0.762, 0.762, 0.0), 'octagon': (0.991, 0.667, 0.0)}
HALF_FOV = math.radians(30.0)
MAX_VIEW_ANGLE = math.radians(65.0)
FRAMES_PER_LOOK = 8


def load_gt(path):
    gt = {}
    for line in open(path):
        line = line.split('#')[0].strip()
        if line:
            i, x, y = (v.strip() for v in line.split(','))
            gt[int(i)] = (float(x), float(y))
    return gt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('shape', choices=['square', 'octagon'])
    ap.add_argument('--loops', type=int, default=3)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', required=True)
    ap.add_argument('--turn-bias', type=float, default=0.03, help='true turn = cmd * (1 + bias)')
    ap.add_argument('--drive-bias', type=float, default=-0.02)
    ap.add_argument('--turn-spread', type=float, default=0.0,
                    help='each turn x uniform(1-spread, 1+spread): erratic turns')
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    gt = load_gt(os.path.join(ROOT, 'ground_truth_hw3.txt'))
    path = load_path(os.path.join(ROOT, f'path_hw3_{a.shape}.txt'))[1:]
    true = list(START[a.shape])
    slam = EkfSlam()
    out = open(a.out, 'w')
    t = 0.0

    def w(rec):
        nonlocal t
        t += 1.0
        rec['t'] = t
        rec['truth'] = list(true)
        out.write(json.dumps(rec) + '\n')

    def look():
        frames = {}
        x, y, th = true
        for i, (lx, ly) in gt.items():
            dx, dy = lx - x, ly - y
            f = math.cos(th) * dx + math.sin(th) * dy
            l = -math.sin(th) * dx + math.cos(th) * dy
            r = math.hypot(f, l)
            view = abs(wrap_angle(math.atan2(dy, dx) + math.pi - math.radians(FACING[i])))
            if f <= 0.3 or r > 3.0 or abs(math.atan2(l, f)) > HALF_FOV or view > MAX_VIEW_ANGLE:
                continue
            sd = 0.005 + 0.015 * r
            phi = wrap_angle(math.radians(FACING[i]) - th)
            phi_bias = rng.normal(0, math.radians(2.0 + 4.0 * r))   # PnP yaw error, per look
            frames[i] = [(f + rng.normal(0, sd), l + rng.normal(0, sd),
                          phi + phi_bias + rng.normal(0, math.radians(1.0)))
                         for _ in range(FRAMES_PER_LOOK)]
        res = fuse_look(slam, frames)
        w({'type': 'look', 'frames': {str(k): v for k, v in frames.items()},
           'status': {str(r[0]): r[4] for r in res}, 'pose': list(slam.pose())})

    def move(kind, value):
        if kind == 'turn':
            true[2] = wrap_angle(true[2] + value * (1 + a.turn_bias)
                                 * rng.uniform(1 - a.turn_spread, 1 + a.turn_spread)
                                 + rng.normal(0, math.radians(1.5)))
            slam.predict_turn(value)
        else:
            d = value * (1 + a.drive_bias) + rng.normal(0, 0.01)
            true[2] = wrap_angle(true[2] + rng.normal(0, math.radians(1.0)))
            true[0] += d * math.cos(true[2])
            true[1] += d * math.sin(true[2])
            slam.predict_drive(value)
        w({'type': 'motion', 'kind': kind, 'value': value})

    w({'type': 'header', 'sim': True, 'shape': a.shape, 'path': path, 'loops': a.loops,
       'camera_offset': 0.10, 'start_truth': START[a.shape]})
    look()
    for loop in range(1, a.loops + 1):
        for goal in path:
            for _ in range(40):
                m = next_motion(slam.pose(), goal, 0.25)
                if m is None:
                    break
                move(*m)
                look()
        w({'type': 'loop', 'loop': loop, 's': slam.s.tolist(), 'P': slam.P.tolist(),
           'ids': slam.ids})
    out.close()
    print(f'wrote {a.out}; true end {np.round(true, 3)}, estimate {np.round(slam.pose(), 3)} '
          f'(start-frame); {len(slam.ids)} landmarks')


if __name__ == '__main__':
    main()
