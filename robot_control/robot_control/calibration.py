"""Measured calibration constants for the rover, and helpers to convert motions to motor commands.

The motor_commands values [L, R] are unitless. These constants map them to physical units.
Measured with tape on the wheels, on the waypoint floor, with a full battery; re-measure if
any of these change. (An earlier calibration without tape gave very different numbers.)
"""

# ---------------------------------------------------------------------------
# Forward motion (Step 1)
# Test: [c, c] for t seconds, distance measured with a tape measure.
#
#   cmd   time   distance   drift
#   0.2   3.0s    70 cm     1.0 cm right
#   0.3   3.0s   131 cm     8.0 cm right
#   0.4   3.0s   196 cm    11.0 cm right
#   0.3   1.5s    70 cm     6.5 cm right
#
# Model: distance = v(c) * (t + COAST_TIME), with v(c) = FORWARD_GAIN * c - FORWARD_OFFSET
# The two durations at 0.3 give the steady speed (61 cm in the extra 1.5s = 0.407 m/s) and
# show the robot goes ~9 cm further than v * t: it keeps coasting after the stop command.
# COAST_TIME = 9 cm / 0.407 m/s = 0.22s, assumed the same at every command.
# Fit error is under 1.3 cm at every test point.
# Commands below FORWARD_OFFSET / FORWARD_GAIN (~0.09) barely move the robot (Step 3).
#
# Drift: always to the right, 1-11 cm. 0.2 drifted the least, so the waypoint follower
# drives at 0.2.
# ---------------------------------------------------------------------------
FORWARD_GAIN = 1.956    # m/s per unit of command
FORWARD_OFFSET = 0.176  # m/s lost to the deadband
COAST_TIME = 0.22       # s, extra distance after stopping = v * COAST_TIME

# ---------------------------------------------------------------------------
# Rotation in place (Step 2)
# Test: [-0.5, 0.5] (left) and [0.5, -0.5] (right), angle measured with floor marks.
#
#   direction        time   measured
#   left  (CCW, +)   0.5s   135 deg
#   left  (CCW, +)   1.0s   270 deg
#   right (CW,  -)   0.5s   140 deg
#   right (CW,  -)   1.0s   285 deg
#
# The angle is proportional to time (no start-up loss or coast), so the rate is a
# least-squares fit through zero: left 270 deg/s, right 284 deg/s (~5% faster).
# Only cmd 0.5 was measured, so always turn at TURN_CMD and control the angle with time.
# Effective wheel base = 2 * v_wheel / omega = 2 * 0.802 / 4.83 = 0.33 m. This is wider
# than the real wheel spacing because the skid-steer wheels slip sideways when turning.
# ---------------------------------------------------------------------------
TURN_CMD = 0.5                # always turn in place with [-TURN_CMD, TURN_CMD]
TURN_RATE_LEFT = 4.712        # rad/s at TURN_CMD, counter-clockwise (270 deg/s)
TURN_RATE_RIGHT = 4.957       # rad/s at TURN_CMD, clockwise (284 deg/s)

# ---------------------------------------------------------------------------
# Deadband (Step 3)
# Test: [0.08, 0.08] for 2.0s moved 2 cm, the smallest forward command that moved.
#       [-0.15, 0.15] for 2.0s turned 1 deg, the smallest turn command that moved.
# The forward model predicts no motion at 0.08, so it is not accurate near the deadband:
# drive at 0.2 or above, where it was measured.
# ---------------------------------------------------------------------------
MIN_FORWARD_CMD = 0.08        # smallest [c, c] that moves the robot
MIN_TURN_CMD = 0.15           # smallest [-c, c] that turns the robot
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
    """Expected steady straight-line speed in m/s for a [c, c] command"""
    sign = 1.0 if c > 0 else -1.0
    return sign * max(0.0, FORWARD_GAIN * abs(c) - FORWARD_OFFSET)


def turn_command(angle):
    """[L, R] and duration in seconds to turn in place by angle radians (+ = left/CCW)"""
    if angle >= 0.0:
        return (-TURN_CMD, TURN_CMD), angle / TURN_RATE_LEFT
    return (TURN_CMD, -TURN_CMD), -angle / TURN_RATE_RIGHT


def drive_command(distance, cmd=0.2):
    """[L, R] and duration in seconds to drive straight by distance meters (negative = backwards)

    The duration is shortened by COAST_TIME because the robot keeps rolling after the stop.
    """
    cmd = max(MIN_DRIVE_CMD, min(abs(cmd), MAX_CMD))
    sign = 1.0 if distance >= 0.0 else -1.0
    duration = abs(distance) / forward_command_to_speed(cmd) - COAST_TIME
    return (sign * cmd, sign * cmd), max(0.0, duration)
