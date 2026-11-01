"""Landmark localization: turn AprilTag sightings into the robot pose (x, y, theta).

Pure Python (no ROS), so it can be tested off the robot.

Frames:
    world   x toward waypoint (1, 0), y to the left, theta counter-clockwise from +x.
            Origin at the start mark.
    robot   origin at the turning centre (middle of the wheels), x forward, y left.
            The floor marks are under this point.
    camera  apriltag_ros optical frame "camera_frame": x right, y down, z forward.
    tag     z points out of the printed face (toward whoever reads the tag).

Noise handling (a small Kalman filter with x, y and theta treated as independent):
    predict  every motion adds variance: drives grow the position variance, turns grow
             the heading variance (HW1 turns repeat to about +-5 deg).
    correct  at each stop the camera's sightings of each tag over ~1 s are reduced to
             their median (rejects single-frame outliers, e.g. PnP pose flips), then fused
             with the prediction weighted by the variances. The measurement variance grows
             with distance to the tag. Heading is fused first; the position measurement then
             uses the fused heading, so a noisy tag yaw is not multiplied by the range.
    gate     a sighting that disagrees with the prediction by more than 3 sigma (and more
             than the minimum gates below) is dropped instead of fused. The gate widens with
             the prediction's variance, so after a long stretch without tags a correct
             sighting is still accepted.
"""
import math
import statistics


# Distance from the turning centre forward to the camera lens, in meters (measured on the robot)
CAMERA_OFFSET = 0.10

# Measurement noise (1 sigma) for one tag, as a function of its distance r in meters
HEADING_STD_BASE = math.radians(2.0)
HEADING_STD_PER_M = math.radians(3.0)
POSITION_STD_BASE = 0.01
POSITION_STD_PER_M = 0.02

# Motion noise (1 sigma) added by each motion
DRIVE_STD_PER_M = 0.05                  # along-track, HW1 drives were within ~2 cm per meter
DRIVE_HEADING_STD = math.radians(1.5)   # heading drift during a drive
TURN_STD_FRACTION = 0.06                # +-5 deg on a 90 deg turn
TURN_STD_BASE = math.radians(1.0)

# Gates: sightings further than GATE_SIGMAS sigma (and the minimum) from the prediction are dropped
GATE_SIGMAS = 3.0
HEADING_GATE = math.radians(30.0)   # minimum
POSITION_GATE = 0.40                # m, minimum


def wrap_angle(a):
    """Wrap an angle to [-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def load_landmarks(path):
    """Read 'id, x, y, facing_deg' lines into {id: (x, y, facing_rad)}; '#' starts a comment.

    facing is the direction the printed face points, in the world frame.
    """
    landmarks = {}
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if line:
                tag_id, x, y, facing = (v.strip() for v in line.split(','))
                landmarks[int(tag_id)] = (float(x), float(y), math.radians(float(facing)))
    return landmarks


def tag_in_robot_frame(translation, quaternion):
    """Camera->tag transform to (forward, left, phi) in the robot frame.

    translation: (x, y, z) of the tag in camera_frame
    quaternion:  (x, y, z, w) rotation of the tag frame in camera_frame
    phi:         direction the tag's face points, relative to the robot heading
                 (pi when the robot looks straight at the tag)
    """
    tx, _, tz = translation
    qx, qy, qz, qw = quaternion
    # tag z axis (third column of the rotation matrix) in camera coordinates
    nx = 2.0 * (qx * qz + qw * qy)
    nz = 1.0 - 2.0 * (qx * qx + qy * qy)
    # camera z forward -> robot x, camera x right -> robot -y
    return tz + CAMERA_OFFSET, -tx, math.atan2(-nx, nz)


def median_sighting(sightings):
    """Median of a list of (forward, left, phi) sightings of one tag"""
    forward = statistics.median(s[0] for s in sightings)
    left = statistics.median(s[1] for s in sightings)
    ref = sightings[0][2]   # take the angle median around one sample so +-pi does not split it
    phi = wrap_angle(ref + statistics.median(wrap_angle(s[2] - ref) for s in sightings))
    return forward, left, phi


class PoseEstimate:
    """Robot pose with a variance for position (same for x and y) and for heading"""

    def __init__(self, x, y, theta, std_xy=0.02, std_theta=math.radians(2.0)):
        self.x, self.y, self.theta = x, y, theta
        self.var_xy = std_xy ** 2
        self.var_theta = std_theta ** 2

    def pose(self):
        return self.x, self.y, self.theta

    def predict_turn(self, angle):
        self.theta = wrap_angle(self.theta + angle)
        self.var_theta += (TURN_STD_FRACTION * abs(angle) + TURN_STD_BASE) ** 2

    def predict_drive(self, distance):
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        # along-track error plus the sideways error from driving with an uncertain heading
        self.var_xy += (DRIVE_STD_PER_M * distance) ** 2 + distance ** 2 * self.var_theta
        self.var_theta += DRIVE_HEADING_STD ** 2

    def correct(self, landmark, sighting):
        """Fuse one tag sighting (forward, left, phi). Returns a short status string."""
        lx, ly, facing = landmark
        forward, left, phi = sighting
        r = math.hypot(forward, left)

        # heading: the tag faces `facing` in the world and `phi` relative to the robot
        theta_meas = wrap_angle(facing - phi)
        innovation = wrap_angle(theta_meas - self.theta)
        var_meas = (HEADING_STD_BASE + HEADING_STD_PER_M * r) ** 2
        if abs(innovation) > max(HEADING_GATE, GATE_SIGMAS * math.sqrt(self.var_theta + var_meas)):
            return f'rejected (heading off by {math.degrees(innovation):+.0f} deg)'
        k = self.var_theta / (self.var_theta + var_meas)
        self.theta = wrap_angle(self.theta + k * innovation)
        self.var_theta *= 1.0 - k

        # position: step back from the tag along the sighting, using the fused heading
        c, s = math.cos(self.theta), math.sin(self.theta)
        x_meas = lx - (c * forward - s * left)
        y_meas = ly - (s * forward + c * left)
        var_meas = (POSITION_STD_BASE + POSITION_STD_PER_M * r) ** 2 + r * r * self.var_theta
        off = math.hypot(x_meas - self.x, y_meas - self.y)
        if off > max(POSITION_GATE, GATE_SIGMAS * math.sqrt(2.0 * (self.var_xy + var_meas))):
            return f'rejected (position off by {off:.2f} m)'
        k = self.var_xy / (self.var_xy + var_meas)
        self.x += k * (x_meas - self.x)
        self.y += k * (y_meas - self.y)
        self.var_xy *= 1.0 - k
        return f'used (r={r:.2f} m, meas=({x_meas:.3f}, {y_meas:.3f}, {math.degrees(theta_meas):.1f} deg))'
