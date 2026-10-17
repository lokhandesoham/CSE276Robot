"""Fixed-command calibration for the rover.

The robot only ever uses two commands, so their behaviour is repeatable:
    drive straight:  [DRIVE_CMD, DRIVE_CMD]
    turn in place:   [-TURN_CMD, TURN_CMD] (left)  or  [TURN_CMD, -TURN_CMD] (right)
Distance and angle are then controlled only by how long the command is held.

Each motion is calibrated against time with a straight-line fit over several durations:
    distance = DRIVE_SPEED * (t - DRIVE_START_LOSS)
    angle    = TURN_RATE   * (t - TURN_DELAY)          (left and right fitted separately)
The *_LOSS / *_DELAY terms absorb the fixed start-up and coasting effects that make
distance not exactly proportional to time.

Measured with tape on the wheels, on the waypoint floor, with a full battery; re-measure if
any of these change.
"""

DRIVE_CMD = 0.2     # [L, R] = [DRIVE_CMD, DRIVE_CMD]; drifted the least in earlier tests
TURN_CMD = 0.3      # 0.5 turned ~280 deg/s and was not repeatable; 0.3 turns ~80 deg/s

# ---------------------------------------------------------------------------
# Drive straight at DRIVE_CMD (one run each)
#
#   time   distance   drift      model
#   1.5s    41 cm     0          39.4 cm
#   2.5s    64 cm     3 cm R     65.7 cm
#   3.5s    93 cm     6 cm R     92.0 cm
#   4.5s   118 cm     8 cm R    118.3 cm
#
# The line fit crosses zero at t = -0.04s, i.e. no measurable start-up loss, so the
# speed is a least-squares fit through zero: 26.3 cm/s, within 1.6 cm of every run.
# Drift grows to ~7% of the distance (to the right); not corrected yet.
# ---------------------------------------------------------------------------
DRIVE_SPEED = 0.263       # m/s
DRIVE_START_LOSS = 0.0    # s, time lost to starting up

# ---------------------------------------------------------------------------
# Turn in place at TURN_CMD (one run each), angle measured with floor marks
#
#   time   left (CCW)   model     right (CW)   model
#   0.6s    40 deg      39 deg     35 deg      43 deg
#   1.2s    85 deg      86 deg    110 deg     104 deg
#   1.8s   130 deg     132 deg    175 deg     164 deg
#   2.4s   180 deg     179 deg    215 deg     225 deg
#
# Line fit per direction, angle = rate * (t - delay):
#   left:  77.5 deg/s, 0.097s delay, within 2 deg of every run
#   right: 100.8 deg/s, 0.174s delay, only within 11 deg: the right turns were not
#          consistent (the rate between runs varies from 67 to 125 deg/s)
# ---------------------------------------------------------------------------
TURN_RATE_LEFT = 1.353    # rad/s, counter-clockwise (77.5 deg/s)
TURN_DELAY_LEFT = 0.097   # s lost at the start of a left turn
TURN_RATE_RIGHT = 1.760   # rad/s, clockwise (100.8 deg/s)
TURN_DELAY_RIGHT = 0.174  # s lost at the start of a right turn

# IMU is not used: its yaw readings disagreed with the floor measurements.


def turn_command(angle):
    """[L, R] and duration in seconds to turn in place by angle radians (+ = left/CCW)"""
    if angle >= 0.0:
        return (-TURN_CMD, TURN_CMD), angle / TURN_RATE_LEFT + TURN_DELAY_LEFT
    return (TURN_CMD, -TURN_CMD), -angle / TURN_RATE_RIGHT + TURN_DELAY_RIGHT


def drive_command(distance):
    """[L, R] and duration in seconds to drive straight by distance meters (negative = backwards)"""
    sign = 1.0 if distance >= 0.0 else -1.0
    duration = abs(distance) / DRIVE_SPEED + DRIVE_START_LOSS
    return (sign * DRIVE_CMD, sign * DRIVE_CMD), duration


def expected_drive(duration):
    """Distance in meters the model predicts for DRIVE_CMD held for duration seconds"""
    return max(0.0, DRIVE_SPEED * (duration - DRIVE_START_LOSS))


def expected_turn(duration, left):
    """Angle in radians the model predicts for TURN_CMD held for duration seconds"""
    if left:
        return max(0.0, TURN_RATE_LEFT * (duration - TURN_DELAY_LEFT))
    return max(0.0, TURN_RATE_RIGHT * (duration - TURN_DELAY_RIGHT))
