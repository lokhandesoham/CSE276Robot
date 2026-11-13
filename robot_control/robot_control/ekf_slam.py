"""EKF-SLAM with point landmarks (AprilTag centres), pure numpy (no ROS: also runs on the Mac).

State     s = [x, y, theta, L1x, L1y, ..., Lnx, Lny]   (map frame = the robot's start pose)
Covariance P, (3 + 2n) x (3 + 2n), with all robot-landmark and landmark-landmark correlations.

Controls (stop-and-go, one motion at a time):
    drive d:  x += d cos(theta), y += d sin(theta)
    turn  a:  theta += a
Measurement of landmark j (the tag centre in the robot frame, forward f and left l):
    f =  cos(theta) dx + sin(theta) dy,   l = -sin(theta) dx + cos(theta) dy,
    with dx = Ljx - x, dy = Ljy - y
Data association: the AprilTag id. A new id augments the state; nothing assumes how many
landmarks exist.
"""
import math

import numpy as np


# ---- noise model (see the report: Q from the HW1/HW2 motion calibration, R measured) ----
DRIVE_STD_BASE = 0.005                  # m
DRIVE_STD_PER_M = 0.05                  # along-track, 5 % of the distance
DRIVE_HEADING_STD_PER_M = math.radians(3.0)   # heading drift while driving
TURN_STD_BASE = math.radians(1.0)
TURN_STD_FRACTION = 0.06                # 6 % of the turn angle

MEAS_STD_BASE = 0.01                    # m
MEAS_STD_PER_M = 0.02                   # grows with range (PnP depth gets worse with distance)

GATE_CHI2 = 13.8                        # Mahalanobis gate, chi^2 with 2 dof at 99.9 %


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def meas_std(r, base=None, per_m=None):
    base = MEAS_STD_BASE if base is None else base
    per_m = MEAS_STD_PER_M if per_m is None else per_m
    return base + per_m * r


class EkfSlam:
    def __init__(self, x0=(0.0, 0.0, 0.0), P0=None, meas_base=None, meas_per_m=None):
        self.s = np.array(x0, dtype=float)
        self.P = np.zeros((3, 3)) if P0 is None else np.array(P0, dtype=float)
        self.ids = []                   # landmark ids in state order
        self.index = {}                 # id -> position of Lx in s
        self.meas_base = MEAS_STD_BASE if meas_base is None else meas_base
        self.meas_per_m = MEAS_STD_PER_M if meas_per_m is None else meas_per_m
        self.updates = {}               # id -> number of updates (incl. the first sighting)

    # ---- accessors -----------------------------------------------------------------
    def pose(self):
        return float(self.s[0]), float(self.s[1]), float(self.s[2])

    def pose_cov(self):
        return self.P[:3, :3].copy()

    def landmark(self, tag_id):
        i = self.index[tag_id]
        return self.s[i:i + 2].copy(), self.P[i:i + 2, i:i + 2].copy()

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
        th = self.s[2]
        c, s = math.cos(th), math.sin(th)
        F_r = np.array([[1.0, 0.0, -d * s],
                        [0.0, 1.0, d * c],
                        [0.0, 0.0, 1.0]])
        # noise in (distance, heading)
        W = np.array([[c, 0.0], [s, 0.0], [0.0, 1.0]])
        Q = np.diag([(DRIVE_STD_BASE + DRIVE_STD_PER_M * abs(d)) ** 2,
                     (DRIVE_HEADING_STD_PER_M * abs(d)) ** 2])
        self._predict(F_r, W, Q)    # Jacobians at the old heading, then move the mean
        self.s[0] += d * c
        self.s[1] += d * s

    def predict_turn(self, a):
        F_r = np.eye(3)
        W = np.array([[0.0], [0.0], [1.0]])
        Q = np.array([[(TURN_STD_BASE + TURN_STD_FRACTION * abs(a)) ** 2]])
        self._predict(F_r, W, Q)
        self.s[2] = wrap_angle(self.s[2] + a)

    # ---- measurement ---------------------------------------------------------------
    def R(self, f, l):
        sd = meas_std(math.hypot(f, l), self.meas_base, self.meas_per_m)
        return np.diag([sd * sd, sd * sd])

    def predict_measurement(self, tag_id):
        """Expected (f, l) of a mapped landmark and the Jacobian H (2 x len(s))"""
        x, y, th = self.s[:3]
        i = self.index[tag_id]
        dx, dy = self.s[i] - x, self.s[i + 1] - y
        c, s = math.cos(th), math.sin(th)
        f = c * dx + s * dy
        l = -s * dx + c * dy
        H = np.zeros((2, len(self.s)))
        H[:, :3] = [[-c, -s, l],
                    [s, -c, -f]]
        H[:, i:i + 2] = [[c, s],
                         [-s, c]]
        return np.array([f, l]), H

    def observe(self, tag_id, f, l):
        """Fuse one sighting (forward, left in the robot frame). Returns a status string."""
        if tag_id not in self.index:
            self._add_landmark(tag_id, f, l)
            return 'new'
        z = np.array([f, l])
        z_hat, H = self.predict_measurement(tag_id)
        nu = z - z_hat
        S = H @ self.P @ H.T + self.R(f, l)
        m2 = float(nu @ np.linalg.solve(S, nu))
        if m2 > GATE_CHI2:
            return f'rejected (Mahalanobis {m2:.1f}, innovation {nu[0] * 100:+.1f}/{nu[1] * 100:+.1f} cm)'
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ nu
        self.s[2] = wrap_angle(self.s[2])
        I_KH = np.eye(len(self.s)) - K @ H
        # Joseph form: (I-KH) P (I-KH)^T + K R K^T, keeps P symmetric positive definite
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R(f, l) @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.updates[tag_id] = self.updates.get(tag_id, 0) + 1
        return f'updated (Mahalanobis {m2:.1f}, innovation {nu[0] * 100:+.1f}/{nu[1] * 100:+.1f} cm)'

    def _add_landmark(self, tag_id, f, l):
        """Augment the state: L = p + Rot(theta) z, with covariance from the robot pose and R"""
        x, y, th = self.s[:3]
        c, s = math.cos(th), math.sin(th)
        L = np.array([x + c * f - s * l, y + s * f + c * l])
        Gr = np.array([[1.0, 0.0, -s * f - c * l],
                       [0.0, 1.0, c * f - s * l]])
        Gz = np.array([[c, -s], [s, c]])
        n = len(self.s)
        P_LL = Gr @ self.P[:3, :3] @ Gr.T + Gz @ self.R(f, l) @ Gz.T
        P_Lx = Gr @ self.P[:3, :]           # 2 x n: correlation with robot and old landmarks
        P = np.zeros((n + 2, n + 2))
        P[:n, :n] = self.P
        P[n:, :n] = P_Lx
        P[:n, n:] = P_Lx.T
        P[n:, n:] = P_LL
        self.P = P
        self.s = np.concatenate([self.s, L])
        self.index[tag_id] = n
        self.ids.append(tag_id)
        self.updates[tag_id] = 1


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
MAX_TURN_STEP = math.radians(46.0)  # turns are made in pieces of at most this, with a look after each


def fuse_look(slam, frames, min_frames=MIN_FRAMES, max_range=MAX_RANGE):
    """frames: {tag_id: [(f, l), ...]} from one look. Median per tag, then one EKF update each.
    Returns [(tag_id, n_frames, f, l, status), ...]."""
    out = []
    for tag_id in sorted(frames):
        fl = frames[tag_id]
        if len(fl) < min_frames:
            out.append((tag_id, len(fl), None, None, f'skipped ({len(fl)} frame)'))
            continue
        f = float(np.median([p[0] for p in fl]))
        l = float(np.median([p[1] for p in fl]))
        if math.hypot(f, l) > max_range:
            out.append((tag_id, len(fl), f, l, f'skipped (range {math.hypot(f, l):.2f} m)'))
            continue
        out.append((tag_id, len(fl), f, l, slam.observe(tag_id, f, l)))
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
