"""Re-run the HW3 EKF-SLAM on a robot (or simulator) log and score it against the measured map.

    tools/.venv/bin/python tools/slam_replay.py RUN.jsonl --start X Y THETA_DEG [--out PREFIX]
    tools/.venv/bin/python tools/slam_replay.py NOISE.jsonl --noise

--start   the measured start pose in the arena frame (x, y in m, heading in deg): the SLAM map
          lives in the robot's start frame and is moved into the arena frame with it
--gt      measured landmark map (default ground_truth_hw3.txt)
--camera-offset / --meas-base / --meas-per-m   re-run with other values than on the robot

Outputs PREFIX_map.png (trajectory + landmarks with 95 % covariance ellipses, start frame and
arena frame), PREFIX_landmarks.png (zoom per landmark, estimate after each loop),
PREFIX_loops.png (error and uncertainty per loop), PREFIX_summary.json, and a text summary.
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'robot_control'))
from robot_control import ekf_slam  # noqa: E402
from robot_control.ekf_slam import EkfSlam, ellipse, fuse_look  # noqa: E402

ARENA = 2.438


def load_gt(path):
    gt = {}
    for line in open(path):
        line = line.split('#')[0].strip()
        if line:
            i, x, y = (v.strip() for v in line.split(','))
            gt[int(i)] = np.array([float(x), float(y)])
    return gt


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def best_fit(src, dst):
    """Rigid 2D transform (R, t) minimising |R src + t - dst|^2 (Kabsch)"""
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


def replay(records, cam_offset, meas_base, meas_per_m, kt=1.0, kd=1.0, drift=None):
    """kt, kd: extra turn/drive scale on the logged motions; drift (rad/m) overrides the run's"""
    header = records[0]
    d_off = 0.0 if cam_offset is None else cam_offset - header.get('camera_offset', 0.10)
    slam = EkfSlam(meas_base=meas_base, meas_per_m=meas_per_m,
                   drift=header.get('drift', 0.0) if drift is None else drift)
    traj, truth, snaps, statuses = [slam.pose()], [], [], []
    if 'truth' in header:
        truth.append(header['truth'])
    for r in records:
        if 'truth' in r and r['type'] in ('motion', 'look'):
            truth.append(r['truth'])
        if r['type'] == 'motion':
            if r['kind'] == 'turn':
                slam.predict_turn(kt * r['value'])
            else:
                slam.predict_drive(kd * r['value'])
            traj.append(slam.pose())
        elif r['type'] == 'look':
            if 'raw' in r:      # camera frame (tx, ty, tz, phi): rebuild with the lens offset
                off = header.get('camera_offset', 0.10) + d_off
                frames = {int(k): [(p[2] + off, -p[0], p[3]) for p in v] for k, v in r['raw'].items()}
            else:
                frames = {int(k): [tuple(p) for p in v] for k, v in r['frames'].items()}
            statuses += fuse_look(slam, frames)
            traj.append(slam.pose())
        elif r['type'] == 'loop':
            snaps.append((r['loop'], slam.landmarks(), len(traj)))
    if not snaps or snaps[-1][2] != len(traj):
        snaps.append((len(snaps) + 1 if not snaps else snaps[-1][0] + 0.5, slam.landmarks(), len(traj)))
    return slam, np.array(traj), np.array(truth) if truth else None, snaps, statuses


def noise_report(records):
    per_tag = {}
    for r in records:
        if r['type'] == 'look':
            for k, v in r['frames'].items():
                per_tag.setdefault(int(k), []).extend(v)
    rows = []
    print('tag  frames  range(m)  std_f(cm)  std_l(cm)')
    for k, v in sorted(per_tag.items()):
        a = np.array(v)
        rng = float(np.hypot(a[:, 0], a[:, 1]).mean())
        sf, sl = a[:, 0].std(ddof=1) * 100, a[:, 1].std(ddof=1) * 100
        rows.append((k, len(a), rng, sf, sl))
        print(f'{k:3d}  {len(a):6d}  {rng:8.2f}  {sf:9.2f}  {sl:9.2f}')
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='+')
    ap.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'THETA_DEG'))
    ap.add_argument('--gt', default=os.path.join(ROOT, 'ground_truth_hw3.txt'))
    ap.add_argument('--out')
    ap.add_argument('--camera-offset', type=float)
    ap.add_argument('--meas-base', type=float)
    ap.add_argument('--meas-per-m', type=float)
    ap.add_argument('--noise', action='store_true', help='stationary noise test summary')
    ap.add_argument('--kt', type=float, default=1.0, help='extra turn scale on logged turns')
    ap.add_argument('--kd', type=float, default=1.0, help='extra drive scale on logged drives')
    ap.add_argument('--drift', type=float, help='deg/m, overrides the run\'s drift')
    a = ap.parse_args()

    if a.noise:
        rows = []
        for path in a.log:
            print(f'== {path}')
            rows += noise_report([json.loads(x) for x in open(path)])
        if len(rows) >= 2:
            r = np.array([[1.0, row[2]] for row in rows])
            sd = np.array([max(row[3], row[4]) / 100 for row in rows])
            base, per_m = np.linalg.lstsq(r, sd, rcond=None)[0]
            print(f'fit (larger of f/l per tag): std = {base * 100:.2f} cm + {per_m * 100:.2f} cm/m * range')
            print('note: these are frame-to-frame std; the EKF uses the median of ~8 frames, but '
                  'bias (calibration, tag placement) does not average out, so keep R larger.')
        return

    log = a.log[0]
    records = [json.loads(x) for x in open(log)]
    header = records[0]
    prefix = a.out or os.path.splitext(log)[0]
    sim = header.get('sim', False)
    start = a.start
    if start is None and sim:
        sx, sy, sth = header['start_truth']
        start = (sx, sy, math.degrees(sth))
    if start is None:
        sys.exit('give --start X Y THETA_DEG (the measured start pose in the arena frame)')
    p0, th0 = np.array(start[:2]), math.radians(start[2])
    R0 = rot(th0)

    slam, traj, truth, snaps, statuses = replay(
        records, a.camera_offset, a.meas_base, a.meas_per_m, a.kt, a.kd,
        None if a.drift is None else math.radians(a.drift))
    gt = load_gt(a.gt)
    to_arena = lambda p: p0 + R0 @ p  # noqa: E731

    # ---- errors per loop ----
    summary = {'log': log, 'start': start, 'loops': []}
    print(f'{log}: {sum(1 for r in records if r["type"] == "motion")} motions, '
          f'{sum(1 for r in records if r["type"] == "look")} looks, '
          f'{len(slam.ids)} landmarks found: {sorted(slam.ids)}')
    n_rej = sum(1 for s in statuses if s[4].startswith('rejected'))
    n_upd = sum(1 for s in statuses if s[4].startswith('updated'))
    print(f'updates {n_upd}, rejected {n_rej}, new {sum(1 for s in statuses if s[4] == "new")}')
    print(f'lost-robot recoveries {slam.recoveries}')
    for loop, lms, _ in snaps:
        ids = sorted(i for i in lms if i in gt)
        est = np.array([to_arena(lms[i][0]) for i in ids])
        ref = np.array([gt[i] for i in ids])
        err = np.linalg.norm(est - ref, axis=1)
        sig = np.array([math.sqrt(np.trace(lms[i][1]) / 2) for i in ids])
        R, t = best_fit(np.array([lms[i][0] for i in ids]), ref) if len(ids) >= 2 else (np.eye(2), 0)
        err_fit = np.linalg.norm(np.array([R @ lms[i][0] + t for i in ids]) - ref, axis=1)
        row = {'loop': loop, 'ids': ids, 'err_cm': (err * 100).round(1).tolist(),
               'mean_err_cm': float(err.mean() * 100), 'mean_err_fit_cm': float(err_fit.mean() * 100),
               'mean_std_cm': float(sig.mean() * 100),
               'fit_rotation_deg': float(math.degrees(math.atan2(R[1, 0], R[0, 0])))}
        summary['loops'].append(row)
        print(f'after loop {loop}: mean error {row["mean_err_cm"]:.1f} cm (measured start pose), '
              f'{row["mean_err_fit_cm"]:.1f} cm (best-fit alignment), mean std {row["mean_std_cm"]:.1f} cm')
    final = summary['loops'][-1]
    print('\nlandmark  est x,y (arena)      truth x,y        error   std x/y (cm)')
    for i in sorted(slam.ids):
        L, C = slam.landmark(i)
        Lw, Cw = to_arena(L), R0 @ C @ R0.T
        g = gt.get(i)
        e = f'{np.linalg.norm(Lw - g) * 100:5.1f} cm' if g is not None else '   n/a'
        gs = f'({g[0]:.3f}, {g[1]:.3f})' if g is not None else ' ' * 14
        print(f'  {i:3d}    ({Lw[0]:.3f}, {Lw[1]:.3f})   {gs}   {e}   '
              f'{math.sqrt(Cw[0, 0]) * 100:.1f}/{math.sqrt(Cw[1, 1]) * 100:.1f}')
    pc = slam.pose_cov()
    summary['final_pose_std'] = [math.sqrt(pc[0, 0]), math.sqrt(pc[1, 1]), math.sqrt(pc[2, 2])]
    if truth is not None:
        te = np.linalg.norm(np.array([to_arena(np.array(p[:2])) for p in traj]) - truth[:, :2], axis=1)
        summary['sim_traj_err_cm'] = [float(te.mean() * 100), float(te.max() * 100)]
        print(f'sim: trajectory error mean {te.mean() * 100:.1f} cm, max {te.max() * 100:.1f} cm')

    # ---- figure 1: map + trajectory ----
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.8))
    for ax, frame in zip(axes, ('start', 'arena')):
        T = (lambda p: p) if frame == 'start' else to_arena
        RT = np.eye(2) if frame == 'start' else R0
        xy = np.array([T(p[:2]) for p in traj])
        bounds = [len(xy)] if not snaps else [s[2] for s in snaps]
        lo = 0
        for k, hi in enumerate(bounds):
            ax.plot(xy[lo:hi, 0], xy[lo:hi, 1], '.-', ms=3, lw=1, label=f'estimated path, loop {k + 1}')
            lo = max(hi - 1, 0)
        th = traj[0, 2] + (0 if frame == 'start' else th0)
        ax.plot(*xy[0], 'k^', ms=9, label='start')
        ax.arrow(*xy[0], 0.15 * math.cos(th), 0.15 * math.sin(th), head_width=0.04, color='k')
        if truth is not None and frame == 'arena':
            ax.plot(truth[:, 0], truth[:, 1], '-', color='0.6', lw=1, label='true path (sim)')
        for i in slam.ids:
            L, C = slam.landmark(i)
            Lp, Cp = T(L), RT @ C @ RT.T
            ex, ey = ellipse(Cp)
            ax.plot(Lp[0] + ex, Lp[1] + ey, 'r-', lw=1)
            ax.plot(*Lp, 'r+', ms=10)
            ax.annotate(str(i), Lp, xytext=(5, 5), textcoords='offset points', color='r')
        if frame == 'arena':
            for i, g in gt.items():
                ax.plot(*g, 'gs', mfc='none', ms=10)
            ax.plot([0, ARENA, ARENA, 0, 0], [0, 0, ARENA, ARENA, 0], 'k--', lw=0.8)
            ax.plot([], [], 'gs', mfc='none', label='measured landmark (ground truth)')
            ax.set_title('Arena frame (SLAM map moved by the measured start pose)')
        else:
            ax.set_title('SLAM output, map frame = robot start pose')
        ax.plot([], [], 'r+-', label='SLAM landmark + 95 % ellipse')
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.legend(fontsize=7, loc='upper right' if frame == 'start' else 'center')
    fig.tight_layout()
    fig.savefig(prefix + '_map.png', dpi=150)

    # ---- figure 2: zoom on each landmark ----
    ids = sorted(slam.ids)
    cols = 4
    fig, axs = plt.subplots(math.ceil(len(ids) / cols), cols, figsize=(12, 3.2 * math.ceil(len(ids) / cols)),
                            squeeze=False)
    for ax, i in zip(axs.flat, ids):
        for loop, lms, _ in snaps:
            if i not in lms:
                continue
            L, C = lms[i]
            Lw, Cw = to_arena(L), R0 @ C @ R0.T
            ex, ey = ellipse(Cw)
            line, = ax.plot((Lw[0] + ex) * 100, (Lw[1] + ey) * 100, lw=1, label=f'loop {loop}')
            ax.plot(Lw[0] * 100, Lw[1] * 100, '+', color=line.get_color())
        if i in gt:
            ax.plot(gt[i][0] * 100, gt[i][1] * 100, 'gs', mfc='none', ms=10, label='truth')
        ax.set_title(f'tag {i}')
        ax.set_aspect('equal', 'datalim')
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=7)
        ax.legend(fontsize=6)
    for ax in list(axs.flat)[len(ids):]:
        ax.axis('off')
    fig.suptitle('Landmark estimates (arena frame, cm) with 95 % covariance ellipses after each loop')
    fig.tight_layout()
    fig.savefig(prefix + '_landmarks.png', dpi=150)

    # ---- figure 3: per loop ----
    fig, ax = plt.subplots(figsize=(5.5, 3.5))
    L = [r['loop'] for r in summary['loops']]
    ax.plot(L, [r['mean_err_cm'] for r in summary['loops']], 'o-', label='mean error (measured start pose)')
    ax.plot(L, [r['mean_err_fit_cm'] for r in summary['loops']], 's-', label='mean error (best-fit alignment)')
    ax.plot(L, [r['mean_std_cm'] for r in summary['loops']], '^-', label='mean landmark std (EKF)')
    ax.set_xlabel('after loop')
    ax.set_ylabel('cm')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(prefix + '_loops.png', dpi=150)

    summary['final'] = final
    summary['noise'] = {k: getattr(ekf_slam, k) for k in ('MEAS_STD_BASE', 'MEAS_STD_PER_M')}
    json.dump(summary, open(prefix + '_summary.json', 'w'), indent=1)
    print(f'\nwrote {prefix}_map.png, _landmarks.png, _loops.png, _summary.json')


if __name__ == '__main__':
    main()
