"""Plot an HW2 run log as the report's 2D trajectory: one dot per 0.5 s of estimated pose.

Usage (on the Mac, from the repo root, after copying ~/hw2_runs/run_*.csv from the robot):
    tools/.venv/bin/python tools/plot_trajectory.py RUN.csv                 # writes RUN.png
    tools/.venv/bin/python tools/plot_trajectory.py RUN.csv --period 1.0    # one dot per second

Dots are the follower's own estimate (prediction while moving, camera fix while stopped).
Also draws the waypoints, the landmarks from landmarks.txt and where each fix was taken, and
prints the total distance and rotation for the report.
"""
import argparse
import csv
import math
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


REPO = pathlib.Path(__file__).resolve().parent.parent


def load_rows(path):
    with open(path) as f:
        return [dict(r, t=float(r['t']), x=float(r['x']), y=float(r['y']), theta=float(r['theta']))
                for r in csv.DictReader(f)]


def load_points(path, columns):
    points = []
    for line in open(path):
        line = line.split('#')[0].strip()
        if line:
            points.append([float(v) for v in line.split(',')][:columns])
    return points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log')
    parser.add_argument('--period', type=float, default=0.5, help='seconds between dots')
    parser.add_argument('--waypoints', default=REPO / 'waypoints_hw2.txt')
    parser.add_argument('--landmarks', default=REPO / 'landmarks.txt')
    args = parser.parse_args()

    rows = load_rows(args.log)
    # the estimate at every `period` seconds: the last row at or before each tick
    dots, i = [], 0
    for k in range(int(rows[-1]['t'] / args.period) + 1):
        while i + 1 < len(rows) and rows[i + 1]['t'] <= k * args.period:
            i += 1
        dots.append(rows[i])
    fixes = [r for r in rows if r['event'] == 'fix' and r['info']]

    distance = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
                   for a, b in zip(rows, rows[1:]) if a['event'] == b['event'] == 'move')
    rotation = sum(abs(math.atan2(math.sin(b['theta'] - a['theta']), math.cos(b['theta'] - a['theta'])))
                   for a, b in zip(rows, rows[1:]) if a['event'] == b['event'] == 'move')

    fig, ax = plt.subplots(figsize=(6, 7))
    ax.axhline(0, color='black', lw=1)
    ax.axvline(0, color='black', lw=1)
    wps = load_points(args.waypoints, 3)
    ax.plot([w[0] for w in wps], [w[1] for w in wps], '--', color='gray', lw=1, label='reference path')
    ax.scatter([w[0] for w in wps], [w[1] for w in wps], marker='x', s=120, color='black', zorder=3,
               label='waypoints')
    for tag_id, x, y, facing in load_points(args.landmarks, 4):
        f = math.radians(facing)
        ax.plot([x - 0.07 * math.sin(f), x + 0.07 * math.sin(f)],
                [y + 0.07 * math.cos(f), y - 0.07 * math.cos(f)], color='tab:blue', lw=4)
        ax.annotate(f'tag {int(tag_id)}', (x, y), textcoords='offset points', xytext=(6, 6))
    ax.scatter([d['x'] for d in dots], [d['y'] for d in dots], s=14, color='tab:red', zorder=4,
               label=f'estimated pose every {args.period:g} s')
    ax.scatter([r['x'] for r in fixes], [r['y'] for r in fixes], s=40, facecolors='none',
               edgecolors='tab:green', zorder=5, label='camera fix')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f'{pathlib.Path(args.log).stem}: {distance:.2f} m, {math.degrees(rotation):.0f} deg, '
                 f'{rows[-1]["t"]:.0f} s', fontsize=10)

    out = pathlib.Path(args.log).with_suffix('.png')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Driven {distance:.2f} m, turned {math.degrees(rotation):.0f} deg, {rows[-1]["t"]:.0f} s')
    for r in rows:
        if r['event'] == 'waypoint':
            print(f'{r["info"]}: estimate ({r["x"]:+.3f}, {r["y"]:+.3f}, {math.degrees(r["theta"]):+.1f} deg)')
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
