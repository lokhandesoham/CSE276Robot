import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray

from robot_control.calibration import (
    DRIVE_L, DRIVE_R, TURN_CMD, expected_drive, expected_turn)


COUNTDOWN_S = 3
PUBLISH_PERIOD_S = 0.05 


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def wait(self, seconds):
        """Keep processing ROS callbacks for the given time"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    argv = remove_ros_args(sys.argv)[1:]
    if len(argv) == 2 and argv[0] in ('drive', 'left', 'right'):
        mode, duration = argv[0], float(argv[1])
        L, R = {'drive': (DRIVE_L, DRIVE_R),
                'left': (-TURN_CMD, TURN_CMD),
                'right': (TURN_CMD, -TURN_CMD)}[mode]
    elif len(argv) == 3:
        mode = None
        L, R, duration = (float(a) for a in argv)
    else:
        print(__doc__, file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        print('Waiting for motor_control to subscribe to motor_commands...')
        while node.publisher.get_subscription_count() == 0:
            node.wait(0.1)

        for i in range(COUNTDOWN_S, 0, -1):
            print(f'Starting in {i}...')
            node.wait(1.0)

        print(f'GO: L={L:.3f}, R={R:.3f} for {duration:.2f}s')

        start = time.monotonic()
        end = start + duration
        while time.monotonic() < end:
            node.send(L, R)
            node.wait(min(PUBLISH_PERIOD_S, end - time.monotonic()))
        actual_duration = time.monotonic() - start

        for _ in range(5):
            node.send(0.0, 0.0)
            node.wait(PUBLISH_PERIOD_S)

        print('STOPPED')
        print('========================================')
        print(f'Command : L={L:.3f}, R={R:.3f}')
        print(f'Duration: {actual_duration:.2f}s')
        if mode == 'drive':
            print(f'Model   : {expected_drive(duration) * 100:.1f} cm')
        elif mode in ('left', 'right'):
            print(f'Model   : {math.degrees(expected_turn(duration, mode == "left")):.1f} deg')
        print('========================================')
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
