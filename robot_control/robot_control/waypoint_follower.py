"""Waypoint follower: drive through a list of (x, y, theta) waypoints with turn-drive-turn.

Usage:
    ros2 run robot_control waypoint_follower
    ros2 run robot_control waypoint_follower --ros-args -p waypoints_file:=/path/to/waypoints.txt
    ros2 run robot_control waypoint_follower --ros-args -p dry_run:=true   # print the plan, don't move
    ros2 run robot_control waypoint_follower --ros-args -p settle_time:=2.0   # longer pause between motions

Algorithm (open loop, dead reckoning):
    For each waypoint, starting from the estimated pose (x, y, theta):
      1. turn in place to face the waypoint     angle = atan2(dy, dx) - theta
      2. drive straight to it                   distance = hypot(dx, dy)
      3. turn in place to the waypoint heading  angle = theta_goal - theta
    Each motion is converted to an [L, R] command and a duration with the calibrated
    model in calibration.py. The estimated pose is updated with the unicycle model:
      x += d * cos(theta),  y += d * sin(theta),  theta += angle
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from robot_control.calibration import drive_command, turn_command


PUBLISH_PERIOD_S = 0.05  # 20Hz, same as the other controllers
# Pause between motions so each one starts from rest, like in calibration. With 0.5s the turn
# after a drive came up short (the robot was still settling); with a 3s pause it was correct.
DEFAULT_SETTLE_S = 1.5
MIN_DISTANCE = 0.01      # m, skip driving shorter than this
MIN_ANGLE = 0.02         # rad (~1 deg), skip turning smaller than this

DEFAULT_WAYPOINTS = os.path.expanduser('~/ros2_ws/src/rubikpi_ros2/waypoints.txt')


def wrap_angle(a):
    """Wrap an angle to [-pi, pi] so turns always take the short way round"""
    return math.atan2(math.sin(a), math.cos(a))


def load_waypoints(path):
    """Read 'x,y,theta' lines into a list of (x, y, theta) tuples"""
    waypoints = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                x, y, theta = (float(v) for v in line.split(','))
                waypoints.append((x, y, theta))
    return waypoints


def plan_motions(waypoints):
    """Turn the waypoint list into a list of ('turn', angle) / ('drive', distance) motions.

    The first waypoint is the start pose; the robot must be placed there.
    """
    motions = []
    x, y, theta = waypoints[0]
    for i, (gx, gy, gtheta) in enumerate(waypoints[1:], start=1):
        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)

        if distance > MIN_DISTANCE:
            # 1. face the waypoint
            heading = math.atan2(dy, dx)
            motions.append((i, 'turn', wrap_angle(heading - theta)))
            theta = heading
            # 2. drive to it (unicycle model update)
            motions.append((i, 'drive', distance))
            x += distance * math.cos(theta)
            y += distance * math.sin(theta)

        # 3. turn to the waypoint heading
        motions.append((i, 'turn', wrap_angle(gtheta - theta)))
        theta = gtheta

    # drop tiny turns
    return [m for m in motions if not (m[1] == 'turn' and abs(m[2]) < MIN_ANGLE)]


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.waypoints_file = self.declare_parameter('waypoints_file', DEFAULT_WAYPOINTS).value
        self.dry_run = self.declare_parameter('dry_run', False).value
        self.settle_time = self.declare_parameter('settle_time', DEFAULT_SETTLE_S).value

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def wait(self, seconds):
        """Keep processing ROS callbacks for the given time"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def execute(self, command, duration):
        """Publish an [L, R] command at 20Hz for duration seconds, then stop and settle"""
        L, R = command
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.send(L, R)
            # stop on time instead of up to one period late; turns are only ~0.3s long
            self.wait(min(PUBLISH_PERIOD_S, end - time.monotonic()))
        self.send(0.0, 0.0)
        self.wait(self.settle_time)

    def run(self):
        waypoints = load_waypoints(self.waypoints_file)
        motions = plan_motions(waypoints)
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {self.waypoints_file}')
        self.get_logger().info(f'Start pose: {waypoints[0]}, place the robot there facing theta')
        self.get_logger().info(f'Pause between motions: {self.settle_time:.1f}s')

        if not self.dry_run:
            self.get_logger().info('Waiting for motor_control to subscribe to motor_commands...')
            while self.publisher.get_subscription_count() == 0:
                self.wait(0.1)
            for i in range(3, 0, -1):
                self.get_logger().info(f'Starting in {i}...')
                self.wait(1.0)

        total_time = 0.0
        for waypoint_index, kind, value in motions:
            if kind == 'turn':
                command, duration = turn_command(value)
                desc = f'turn {math.degrees(value):+.1f} deg'
            else:
                command, duration = drive_command(value)
                desc = f'drive {value * 100:.1f} cm'
            total_time += duration + self.settle_time

            self.get_logger().info(
                f'-> waypoint {waypoint_index} {waypoints[waypoint_index]}: {desc} '
                f'(L={command[0]:+.2f}, R={command[1]:+.2f}, {duration:.2f}s)')
            if not self.dry_run:
                self.execute(command, duration)

        self.get_logger().info(f'Done, {len(motions)} motions, {total_time:.1f}s total')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        node.run()
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
