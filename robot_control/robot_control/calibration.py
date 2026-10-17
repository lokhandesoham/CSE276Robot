"""Measured calibration constants for the rover, and helpers to convert motions to motor commands.

The motor_commands values [L, R] are unitless. These constants map them to physical units.
Measured with tape on the wheels, on the waypoint floor, with a full battery; re-measure if
any of these change. (An earlier calibration without tape gave very different numbers.)
"""

# ---------------------------------------------------------------------------
# Forward motion (Step 1)
# Test: [c, c] for t seconds, distance measured with a tape measure.
#
#   cmd   time    distance   drift
#   0.2   3.0s     70 cm     1.0 cm right
#   0.2   4.43s   112 cm     0 cm          (check run)
#   0.3   3.0s    131 cm     8.0 cm right
#   0.4   3.0s    196 cm    11.0 cm right
#   0.3   1.5s     70 cm     6.5 cm right
#
# Speed is roughly linear in the command over 0.2-0.4 (3s runs):
#   v = FORWARD_GAIN * cmd - FORWARD_OFFSET, so commands below ~0.09 barely move (Step 3).
#
# The waypoint follower only drives at DRIVE_CMD = 0.2, which drifted the least, so that
# command is calibrated directly from its two durations:
#   distance = DRIVE_SPEED * (t - DRIVE_START_LOSS)
#   (112 - 70) cm / (4.43 - 3.0) s = 0.294 m/s, and the line crosses zero at t = 0.62s.
# A first model that assumed every command coasts like 0.3 does (+9 cm) predicted 100 cm
# for the 4.43s run but the robot went 112 cm, so 0.2 has its own constants.
# ---------------------------------------------------------------------------
FORWARD_GAIN = 1.956    # m/s per unit of command (3s runs)
FORWARD_OFFSET = 0.176  # m/s lost to the deadband (3s runs)

DRIVE_CMD = 0.2           # always drive with [DRIVE_CMD, DRIVE_CMD]
DRIVE_SPEED = 0.294       # m/s at DRIVE_CMD
DRIVE_START_LOSS = 0.62   # s, time lost to starting up at DRIVE_CMD

# ---------------------------------------------------------------------------
# Rotation in place (Step 2)
# Test: [-0.5, 0.5] (left) and [0.5, -0.5] (right), angle measured with floor marks.
#
#   direction        time    measured
#   left  (CCW, +)   0.33s    70 deg   (check run)
#   left  (CCW, +)   0.5s    135 deg
#   left  (CCW, +)   1.0s    270 deg
#   right (CW,  -)   0.16s    45 deg   (check run)
#   right (CW,  -)   0.32s    90 deg   (check run)
#   right (CW,  -)   0.5s    140 deg
#   right (CW,  -)   1.0s    285 deg
#
# Right turns are proportional to time: least-squares fit through zero gives 284 deg/s,
# within 2 deg of every run.
# Left turns come up short on short turns, so they are fit with a start-up delay:
#   angle = TURN_RATE_LEFT * (t - TURN_DELAY_LEFT)  ->  292 deg/s, 0.068s delay.
# The left fit is only within ~9 deg, because the turns are so fast that the 20Hz
# motor_control loop (50ms = ~14 deg) limits how repeatable they are.
# Only cmd 0.5 was measured, so always turn at TURN_CMD and control the angle with time.
# Effective wheel base = 2 * v_wheel / omega = 2 * 0.802 / 5.02 = 0.32 m. This is wider
# than the real wheel spacing because the skid-steer wheels slip sideways when turning.
# ---------------------------------------------------------------------------
TURN_CMD = 0.5                # always turn in place with [-TURN_CMD, TURN_CMD]
TURN_RATE_LEFT = 5.097        # rad/s at TURN_CMD, counter-clockwise (292 deg/s)
TURN_DELAY_LEFT = 0.068       # s lost at the start of a left turn
TURN_RATE_RIGHT = 4.952       # rad/s at TURN_CMD, clockwise (284 deg/s)

# ---------------------------------------------------------------------------
# Deadband (Step 3)
# Test: [0.08, 0.08] for 2.0s moved 2 cm, the smallest forward command that moved.
#       [-0.15, 0.15] for 2.0s turned 1 deg, the smallest turn command that moved.
# ---------------------------------------------------------------------------
MIN_FORWARD_CMD = 0.08        # smallest [c, c] that moves the robot
MIN_TURN_CMD = 0.15           # smallest [-c, c] that turns the robot

# IMU is not used: its yaw readings disagreed with the floor measurements.

MAX_CMD = 0.5           # motor_control clips commands to [-0.5, 0.5]


def forward_speed_to_command(v):
    """Command value c for [c, c] that drives straight at about v m/s (negative v = backwards)"""
    if v == 0.0:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    c = (abs(v) + FORWARD_OFFSET) / FORWARD_GAIN
    return sign * min(c, MAX_CMD)


def forward_command_to_speed(c):
    """Approximate straight-line speed in m/s for a [c, c] command"""
    sign = 1.0 if c > 0 else -1.0
    return sign * max(0.0, FORWARD_GAIN * abs(c) - FORWARD_OFFSET)


def turn_command(angle):
    """[L, R] and duration in seconds to turn in place by angle radians (+ = left/CCW)"""
    if angle >= 0.0:
        return (-TURN_CMD, TURN_CMD), angle / TURN_RATE_LEFT + TURN_DELAY_LEFT
    return (TURN_CMD, -TURN_CMD), -angle / TURN_RATE_RIGHT


def drive_command(distance):
    """[L, R] and duration in seconds to drive straight by distance meters (negative = backwards)"""
    sign = 1.0 if distance >= 0.0 else -1.0
    duration = abs(distance) / DRIVE_SPEED + DRIVE_START_LOSS
    return (sign * DRIVE_CMD, sign * DRIVE_CMD), duration
