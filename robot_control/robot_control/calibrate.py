"""Calibration helper: drive the robot at a fixed [L, R] command for a fixed time, then stop.

Usage:
    ros2 run robot_control calibrate L R DURATION
    e.g. ros2 run robot_control calibrate 0.3 0.3 3.0

Measure the distance (or rotation) with a tape measure and floor marks afterwards.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray


COUNTDOWN_S = 3
PUBLISH_PERIOD_S = 0.05  # 20Hz, same as the other controllers


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
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 1
    L, R, duration = (float(a) for a in argv)

    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        # make sure motor_control is running and connected before we start
        print('Waiting for motor_control to subscribe to motor_commands...')
        while node.publisher.get_subscription_count() == 0:
            node.wait(0.1)

        for i in range(COUNTDOWN_S, 0, -1):
            print(f'Starting in {i}...')
            node.wait(1.0)

        print(f'GO: L={L:.3f}, R={R:.3f} for {duration:.2f}s')

        start = time.monotonic()
        while time.monotonic() - start < duration:
            node.send(L, R)
            node.wait(PUBLISH_PERIOD_S)
        actual_duration = time.monotonic() - start

        for _ in range(5):
            node.send(0.0, 0.0)
            node.wait(PUBLISH_PERIOD_S)

        print('STOPPED')
        print('========================================')
        print(f'Command : L={L:.3f}, R={R:.3f}')
        print(f'Duration: {actual_duration:.2f}s')
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
