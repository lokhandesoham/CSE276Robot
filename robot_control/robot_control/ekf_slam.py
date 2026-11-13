"""EKF-SLAM with AprilTag landmarks, pure numpy (no ROS: also runs on the Mac).

State     s = [x, y, theta, L1x, L1y, A1, ..., Lnx, Lny, An]   (map frame = the robot's start pose)
          (Ljx, Ljy) = tag centre, Aj = direction the tag's printed face points
Covariance P, (3 + 3n) x (3 + 3n), with all robot-landmark and landmark-landmark correlations.

Controls (stop-and-go, one motion at a time):
    drive d:  x += d cos(theta), y += d sin(theta)
    turn  a:  theta += a
Measurement of landmark j (from the AprilTag pose in the robot frame):
    f   =  cos(theta) dx + sin(theta) dy        forward distance to the tag centre
    l   = -sin(theta) dx + cos(theta) dy        left distance
    phi = Aj - theta                            tag face direction relative to the robot
    with dx = Ljx - x, dy = Ljy - y
phi makes the robot heading directly observable from a single tag; it is noisy at long range
(PnP), so its noise grows quickly with range and it is not used beyond PHI_MAX_RANGE.
Data association: the AprilTag id. A new id augments the state; nothing assumes how many
landmarks exist.
"""
import math

import numpy as np


# ---- noise model ---------------------------------------------------------------------
# motion (Q): HW1/HW2 calibration. Systematic errors (short turns/drives on a weaker battery,
# curving while driving) are removed first with turn/drive scales and the drift below, fitted
# from the innovations of a run (tools/slam_fit_motion.py); Q covers what is left.
DRIVE_STD_BASE = 0.005                  # m
DRIVE_STD_PER_M = 0.05                  # along-track, 5 % of the distance
DRIVE_HEADING_STD_PER_M = math.radians(4.0)   # heading drift while driving
TURN_STD_BASE = math.radians(2.0)
TURN_STD_FRACTION = 0.35                # turns are erratic on the arena floor: the same command
                                        # came out 0.3-0.9 x planned on the square runs and
                                        # 1.1 x planned on the octagon runs (tag bearings)
TURN_GAIN = 0.9                         # between the two regimes; with +-35 % it covers both

# measurement (R): frame-to-frame jitter is < 1 mm, the error is bias (calibration, lens
# offset, placement), measured against the tape at 1.7 m as ~4 cm
MEAS_STD_BASE = 0.01                    # m
MEAS_STD_PER_M = 0.02                   # grows with range
PHI_STD_BASE = math.radians(3.0)
PHI_STD_PER_M = math.radians(5.0)       # tag yaw from PnP: ~10 deg at 1.5 m in the logs
PHI_MAX_RANGE = 2.2                     # m, farther: position only

GATE_CHI2 = {2: 13.8, 3: 16.3}          # Mahalanobis gate, chi^2 at 99.9 % (2 or 3 dof)
# lost-robot recovery: after this many looks in a row where every known tag was rejected,
# widen the robot pose uncertainty and retry (the AprilTag id makes a wrong match impossible,
# so persistent rejections mean the pose estimate is off, not the measurements)
LOST_LOOKS = 2
LOST_INFLATE = (0.15, 0.15, math.radians(30.0))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class EkfSlam:
    def __init__(self, x0=(0.0, 0.0, 0.0), P0=None, meas_base=None, meas_per_m=None, drift=0.0,
                 turn_gain=None):
        self.s = np.array(x0, dtype=float)
        self.P = np.zeros((3, 3)) if P0 is None else np.array(P0, dtype=float)
        self.ids = []                   # landmark ids in state order
        self.index = {}                 # id -> position of Lx in s (then Ly, A)
        self.meas_base = MEAS_STD_BASE if meas_base is None else meas_base
        self.meas_per_m = MEAS_STD_PER_M if meas_per_m is None else meas_per_m
        self.updates = {}               # id -> number of updates (incl. the first sighting)
        self.lost_looks = 0             # looks in a row with only rejections
        self.drift = drift              # rad/m, heading change while driving (+ = curves left)
        self.turn_gain = TURN_GAIN if turn_gain is None else turn_gain
        self.recoveries = 0

    # ---- accessors -----------------------------------------------------------------
    def pose(self):
        return float(self.s[0]), float(self.s[1]), float(self.s[2])

    def pose_cov(self):
        return self.P[:3, :3].copy()

    def landmark(self, tag_id):
        """(x, y) of a landmark and its 2x2 position covariance"""
        i = self.index[tag_id]
        return self.s[i:i + 2].copy(), self.P[i:i + 2, i:i + 2].copy()

    def landmark_facing(self, tag_id):
        i = self.index[tag_id]
        return float(self.s[i + 2]), float(self.P[i + 2, i + 2])

    def landmarks(self):
        return {t: self.landmark(t) for t in self.ids}

    # ---- prediction ----------------------------------------------------------------
    def _predict(self, F_r, W, Q):
        """Robot block Jacobian F_r (3x3), noise Jacobian W (3xk), control noise Q (kxk).
        F is identity on the landmarks, so only the robot rows/columns change."""
        P = self.P
        P[:3, :] = F_r @ P[:3, :]
        P[:, :3] = P[:, :3] @ F_r.T
        P[:3, :3] += W @ Q @ W.T
        self.P = 0.5 * (P + P.T)

    def predict_drive(self, d):
        # the robot curves by drift*d: position moves along the mean heading of the arc
        th = self.s[2] + 0.5 * self.drift * abs(d)
        c, s = math.cos(th), math.sin(th)
        F_r = np.array([[1.0, 0.0, -d * s],
                        [0.0, 1.0, d * c],
                        [0.0, 0.0, 1.0]])
        W = np.array([[c, 0.0], [s, 0.0], [0.0, 1.0]])     # noise in (distance, heading)
        Q = np.diag([(DRIVE_STD_BASE + DRIVE_STD_PER_M * abs(d)) ** 2,
                     (DRIVE_HEADING_STD_PER_M * abs(d)) ** 2])
        self._predict(F_r, W, Q)    # Jacobians at the old heading, then move the mean
        self.s[0] += d * c
        self.s[1] += d * s
        self.s[2] = wrap_angle(self.s[2] + self.drift * abs(d))

    def predict_turn(self, a):
        """a: the planned turn. The robot turns TURN_GAIN * a on average, +-TURN_STD_FRACTION."""
        a = self.turn_gain * a
        W = np.array([[0.0], [0.0], [1.0]])
        Q = np.array([[(TURN_STD_BASE + TURN_STD_FRACTION * abs(a)) ** 2]])
        self._predict(np.eye(3), W, Q)
        self.s[2] = wrap_angle(self.s[2] + a)

    # ---- measurement ---------------------------------------------------------------
    def R(self, r, use_phi):
        sd = self.meas_base + self.meas_per_m * r
        d = [sd * sd, sd * sd]
        if use_phi:
            d.append((PHI_STD_BASE + PHI_STD_PER_M * r) ** 2)
        return np.diag(d)

    def predict_measurement(self, tag_id, use_phi=True):
        """Expected (f, l[, phi]) of a mapped landmark and the Jacobian H"""
        x, y, th = self.s[:3]
        i = self.index[tag_id]
        dx, dy = self.s[i] - x, self.s[i + 1] - y
        c, s = math.cos(th), math.sin(th)
        f = c * dx + s * dy
        l = -s * dx + c * dy
        m = 3 if use_phi else 2
        H = np.zeros((m, len(self.s)))
        H[:2, :3] = [[-c, -s, l],
                     [s, -c, -f]]
        H[:2, i:i + 2] = [[c, s],
                          [-s, c]]
        z = [f, l]
        if use_phi:
            z.append(wrap_angle(self.s[i + 2] - th))
            H[2, 2] = -1.0
            H[2, i + 2] = 1.0
        return np.array(z), H

    def observe(self, tag_id, f, l, phi=None, retry=True):
        """Fuse one sighting. Returns a status string ('new', 'updated ...', 'rejected ...')."""
        r = math.hypot(f, l)
        use_phi = phi is not None and r <= PHI_MAX_RANGE
        if tag_id not in self.index:
            if phi is None:
                return 'skipped (new tag without orientation)'
            self._add_landmark(tag_id, f, l, phi, r)
            return 'new'
        z_hat, H = self.predict_measurement(tag_id, use_phi)
        z = np.array([f, l, phi] if use_phi else [f, l])
        nu = z - z_hat
        if use_phi:
            nu[2] = wrap_angle(nu[2])
        R = self.R(r, use_phi)
        S = H @ self.P @ H.T + R
        m2 = float(nu @ np.linalg.solve(S, nu))
        text = (f'innovation {nu[0] * 100:+.1f}/{nu[1] * 100:+.1f} cm'
                + (f' {math.degrees(nu[2]):+.1f} deg' if use_phi else ''))
        if m2 > GATE_CHI2[len(z)]:
            return f'rejected (Mahalanobis {m2:.1f}, {text})'
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ nu
        self.s[2] = wrap_angle(self.s[2])
        for j in self.index.values():
            self.s[j + 2] = wrap_angle(self.s[j + 2])
        I_KH = np.eye(len(self.s)) - K @ H
        # Joseph form: (I-KH) P (I-KH)^T + K R K^T, keeps P symmetric positive definite
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.updates[tag_id] = self.updates.get(tag_id, 0) + 1
        return f'updated (Mahalanobis {m2:.1f}, {text})'

    def _add_landmark(self, tag_id, f, l, phi, r):
        """Augment the state: L = p + Rot(theta) z, A = theta + phi, covariance from the
        robot pose and R through the Jacobians Gr (robot) and Gz (measurement)"""
        x, y, th = self.s[:3]
        c, s = math.cos(th), math.sin(th)
        L = np.array([x + c * f - s * l, y + s * f + c * l, wrap_angle(th + phi)])
        Gr = np.array([[1.0, 0.0, -s * f - c * l],
                       [0.0, 1.0, c * f - s * l],
                       [0.0, 0.0, 1.0]])
        Gz = np.array([[c, -s, 0.0],
                       [s, c, 0.0],
                       [0.0, 0.0, 1.0]])
        n = len(self.s)
        P_LL = Gr @ self.P[:3, :3] @ Gr.T + Gz @ self.R(r, True) @ Gz.T
        P_Lx = Gr @ self.P[:3, :]           # 3 x n: correlation with robot and old landmarks
        P = np.zeros((n + 3, n + 3))
        P[:n, :n] = self.P
        P[n:, :n] = P_Lx
        P[:n, n:] = P_Lx.T
        P[n:, n:] = P_LL
        self.P = P
        self.s = np.concatenate([self.s, L])
        self.index[tag_id] = n
        self.ids.append(tag_id)
        self.updates[tag_id] = 1

    def inflate_pose(self, std=LOST_INFLATE):
        self.P[:3, :3] += np.diag([v * v for v in std])
        self.recoveries += 1


def ellipse(cov, chi2=5.991, points=60):
    """(xs, ys) offsets of the confidence ellipse of a 2x2 covariance (default 95 %)"""
    vals, vecs = np.linalg.eigh(cov)
    vals = np.maximum(vals, 0.0)
    t = np.linspace(0.0, 2.0 * math.pi, points)
    circle = np.stack([np.cos(t), np.sin(t)])
    pts = vecs @ (np.sqrt(chi2 * vals)[:, None] * circle)
    return pts[0], pts[1]


# ---- shared by the robot node, the simulator and the Mac replay -------------------------
MIN_FRAMES = 2              # a tag must be seen in this many frames of one look to be used
MAX_RANGE = 3.0             # m, farther sightings are ignored (PnP too noisy)

POS_TOL = 0.06              # m, waypoint reached
APPROACH_RANGE = 0.15       # m, closer than this only drive forward/backward
AIM_TOL = math.radians(6.0)
MIN_DRIVE = 0.04            # m, shorter drives barely move the robot
MAX_TURN_STEP = math.radians(25.0)  # turns are made in pieces of at most this, with a look after each:
                                    # smaller than the +-32 deg field of view, so a mapped tag stays in
                                    # view across a piece and corrects the heading before a new tag is
                                    # added (with 46 deg pieces the octagon lost ~13 deg per vertex)


def circular_median(angles):
    ref = angles[0]
    return wrap_angle(ref + float(np.median([wrap_angle(a - ref) for a in angles])))


def fuse_look(slam, frames, min_frames=MIN_FRAMES, max_range=MAX_RANGE):
    """frames: {tag_id: [(f, l, phi), ...]} from one look. Median per tag, then one EKF update
    each. Returns [(tag_id, n_frames, f, l, status), ...]."""
    out, medians = [], {}
    for tag_id in sorted(frames):
        fl = frames[tag_id]
        if len(fl) < min_frames:
            out.append((tag_id, len(fl), None, None, f'skipped ({len(fl)} frame)'))
            continue
        f = float(np.median([p[0] for p in fl]))
        l = float(np.median([p[1] for p in fl]))
        phi = circular_median([p[2] for p in fl]) if len(fl[0]) > 2 else None
        if math.hypot(f, l) > max_range:
            out.append((tag_id, len(fl), f, l, f'skipped (range {math.hypot(f, l):.2f} m)'))
            continue
        medians[tag_id] = (f, l, phi)
        out.append((tag_id, len(fl), f, l, slam.observe(tag_id, f, l, phi)))

    known = [o for o in out if o[0] in medians and o[4] != 'new']
    if known and all(o[4].startswith('rejected') for o in known):
        slam.lost_looks += 1
        if slam.lost_looks >= LOST_LOOKS:
            # the pose estimate has drifted away: widen its uncertainty and try again
            slam.inflate_pose()
            slam.lost_looks = 0
            for k, o in enumerate(out):
                if o in known:
                    status = slam.observe(o[0], *medians[o[0]])
                    out[k] = o[:4] + (f'recovery: {status}',)
    elif known:
        slam.lost_looks = 0
    return out


def next_motion(pose, goal, step):
    """('turn', angle), ('drive', distance) or None when (x, y) of goal is reached"""
    x, y, theta = pose
    dx, dy = goal[0] - x, goal[1] - y
    dist = math.hypot(dx, dy)
    if dist < POS_TOL:
        return None
    err = wrap_angle(math.atan2(dy, dx) - theta)
    if dist < APPROACH_RANGE:
        along = dist * math.cos(err)
        return ('drive', along) if abs(along) >= MIN_DRIVE else None
    if abs(err) > AIM_TOL:
        return ('turn', max(-MAX_TURN_STEP, min(MAX_TURN_STEP, err)))
    along = dist * math.cos(err)
    return ('drive', along if along < step + APPROACH_RANGE else step)


def load_path(path):
    """Waypoints relative to the start pose, one 'x, y' per line ('#' comments allowed)"""
    pts = []
    with open(path) as fh:
        for line in fh:
            line = line.split('#')[0].strip()
            if line:
                vals = [float(v) for v in line.split(',')]
                pts.append((vals[0], vals[1]))
    return pts
