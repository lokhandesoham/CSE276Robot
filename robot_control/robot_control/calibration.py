DRIVE_CMD = 0.2     
TURN_CMD = 0.3    
DRIVE_SPEED = 0.263       
DRIVE_START_LOSS = 0.0   
DRIVE_TRIM = 0.004        # [L, R] = [DRIVE_CMD - DRIVE_TRIM, DRIVE_CMD + DRIVE_TRIM]
DRIVE_L = DRIVE_CMD - DRIVE_TRIM
DRIVE_R = DRIVE_CMD + DRIVE_TRIM

TURN_RATE_LEFT = 1.353    
TURN_DELAY_LEFT = 0.066   
TURN_RATE_RIGHT = 1.730   
TURN_DELAY_RIGHT = 0.061  

def turn_command(angle):
    """[L, R] and duration in seconds to turn in place by angle radians (+ = left/CCW)"""
    if angle >= 0.0:
        return (-TURN_CMD, TURN_CMD), angle / TURN_RATE_LEFT + TURN_DELAY_LEFT
    return (TURN_CMD, -TURN_CMD), -angle / TURN_RATE_RIGHT + TURN_DELAY_RIGHT


def drive_command(distance):
    """[L, R] and duration in seconds to drive straight by distance meters (negative = backwards)"""
    sign = 1.0 if distance >= 0.0 else -1.0
    duration = abs(distance) / DRIVE_SPEED + DRIVE_START_LOSS
    return (sign * DRIVE_L, sign * DRIVE_R), duration


def expected_drive(duration):
    """Distance in meters the model predicts for DRIVE_CMD held for duration seconds"""
    return max(0.0, DRIVE_SPEED * (duration - DRIVE_START_LOSS))


def expected_turn(duration, left):
    """Angle in radians the model predicts for TURN_CMD held for duration seconds"""
    if left:
        return max(0.0, TURN_RATE_LEFT * (duration - TURN_DELAY_LEFT))
    return max(0.0, TURN_RATE_RIGHT * (duration - TURN_DELAY_RIGHT))
