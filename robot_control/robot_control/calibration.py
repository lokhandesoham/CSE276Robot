"""Measured calibration constants for the rover, and helpers to convert speeds to motor commands.

The motor_commands values [L, R] are unitless. These constants map them to physical units.
All measurements are on the same floor with a full battery; re-measure if either changes.
"""

# ---------------------------------------------------------------------------
# Forward motion (Step 1)
# Test: [c, c] for 3.0s, distance measured with a tape measure.
#
#   cmd   runs  mean distance   mean speed
#   0.2   3     32.0 cm         0.107 m/s
#   0.3   3     53.3 cm         0.178 m/s
#   0.4   1     73.0 cm         0.243 m/s
#
# Linear least-squares fit:  v = FORWARD_GAIN * cmd - FORWARD_OFFSET
# Fit error is under 0.6 cm at every test point, so the response is linear in this range.
# The offset means commands below about FORWARD_OFFSET / FORWARD_GAIN (~0.04) barely move
# the robot. It includes acceleration/stopping losses as well as the true deadband (Step 3).
#
# Drift: the robot ended 3-8 cm left of the line in every run. The sideways error is
# a constant ~11% of the distance travelled, which matches a heading kick at start-up,
# not a steady curve (a steady curve would grow with distance squared).
# ---------------------------------------------------------------------------
FORWARD_GAIN = 0.683    # m/s per unit of command
FORWARD_OFFSET = 0.029  # m/s lost to deadband and start-up

# ---------------------------------------------------------------------------
# Rotation in place (Step 2)
# Test: [-0.5, 0.5] (left) and [0.5, -0.5] (right) for 2.0s, angle measured with floor marks.
#
#   direction        cmd   measured   rate
#   left  (CCW, +)   0.5   175 deg    1.527 rad/s
#   right (CW,  -)   0.5   185 deg    1.614 rad/s
#
# Only cmd 0.5 was measured, so always turn at TURN_CMD and control the angle with time.
# Right turns are ~6% faster than left turns, so each direction has its own rate.
# Effective wheel base = 2 * v_wheel / omega = 2 * 0.313 / 1.571 = 0.40 m. This is much
# wider than the real wheel spacing because the skid-steer wheels slip sideways when turning.
# ---------------------------------------------------------------------------
TURN_CMD = 0.5                # always turn in place with [-TURN_CMD, TURN_CMD]
TURN_RATE_LEFT = 1.527        # rad/s at TURN_CMD, counter-clockwise
TURN_RATE_RIGHT = 1.614       # rad/s at TURN_CMD, clockwise

# ---------------------------------------------------------------------------
# Deadband (Step 3)
# Test: [0.08, 0.08] for 2.0s moved 2 cm, the smallest command that moved the robot.
# The forward model predicts 5 cm here, so it is not accurate near the deadband:
# drive at 0.2 or above, where it was measured.
# ---------------------------------------------------------------------------
MIN_FORWARD_CMD = 0.08        # smallest [c, c] that moves the robot
MIN_DRIVE_CMD = 0.2           # slowest command the forward model is valid for

# IMU is not used: its yaw readings disagreed with the floor measurements.

MAX_CMD = 0.5           # motor_control clips commands to [-0.5, 0.5]


def forward_speed_to_command(v):
    """Command value c for [c, c] that drives straight at v m/s (negative v = backwards)"""
    if v == 0.0:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    c = (abs(v) + FORWARD_OFFSET) / FORWARD_GAIN
    return sign * min(c, MAX_CMD)


def forward_command_to_speed(c):
    """Expected straight-line speed in m/s for a [c, c] command"""
    sign = 1.0 if c > 0 else -1.0
    return sign * max(0.0, FORWARD_GAIN * abs(c) - FORWARD_OFFSET)


def turn_command(angle):
    """[L, R] and duration in seconds to turn in place by angle radians (+ = left/CCW)"""
    if angle >= 0.0:
        return (-TURN_CMD, TURN_CMD), angle / TURN_RATE_LEFT
    return (TURN_CMD, -TURN_CMD), -angle / TURN_RATE_RIGHT


def drive_command(distance, cmd=0.3):
    """[L, R] and duration in seconds to drive straight by distance meters (negative = backwards)"""
    cmd = max(MIN_DRIVE_CMD, min(abs(cmd), MAX_CMD))
    sign = 1.0 if distance >= 0.0 else -1.0
    return (sign * cmd, sign * cmd), abs(distance) / forward_command_to_speed(cmd)
