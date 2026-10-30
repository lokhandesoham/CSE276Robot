"""Closed-Loop Visual Waypoint Controller for CSE 276A HW2.

Closes the control loop around `/robot_pose` from `pose_estimator_node`:
  1. Aligns heading toward target waypoint using EKF pose estimate; verifies heading
     visually after turn settle pause and applies corrective micro-turn if needed.
  2. Splits long drives (> max_step_distance_m) into sub-segments so visual feedback
     eliminates lateral cross-track drift mid-course.
  3. Performs closed-loop arrival correction at each waypoint and locks final heading.
  4. Automatically logs 0.5s trajectory samples (`hw2_trajectory.csv`) and waypoint
     localization errors + total moving distance (`hw2_summary.json`) for the HW2 report.
"""

import csv
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import yaml

from robot_control.calibration import drive_command, turn_command


PUBLISH_PERIOD_S = 0.05
MIN_DISTANCE_M = 0.015
MIN_ANGLE_RAD = 0.025


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def resolve_default_config_path():
    try:
        share_dir = get_package_share_directory('robot_control')
        candidate = os.path.join(share_dir, 'config', 'hw2_config.yaml')
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'hw2_config.yaml')
    )


class ClosedLoopFollowerNode(Node):
    def __init__(self):
        super().__init__('closed_loop_follower_node')

        config_path = self.declare_parameter('config_file', resolve_default_config_path()).value
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.unit_scale_m = float(
            self.declare_parameter('unit_scale_m', self.cfg.get('unit_scale_m', 0.5)).value
        )
        self.dry_run = bool(self.declare_parameter('dry_run', False).value)
        self.log_dir = str(
            self.declare_parameter('log_dir', os.path.abspath('.')).value
        )

        ctrl_cfg = self.cfg.get('controller', {})
        self.pos_tol_m = float(ctrl_cfg.get('pos_tolerance_m', 0.035))
        self.head_tol_rad = float(ctrl_cfg.get('heading_tolerance_rad', 0.05))
        self.max_step_m = float(ctrl_cfg.get('max_step_distance_m', 0.55))
        self.max_corrections = int(ctrl_cfg.get('max_corrections', 2))
        self.settle_time_s = float(ctrl_cfg.get('settle_time_s', 1.5))
        self.log_period_s = float(ctrl_cfg.get('trajectory_log_period_s', 0.5))

        # Parse waypoints in units and convert to meters
        raw_wps = self.cfg.get(
            'waypoints_units',
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 2.0, math.pi], [0.0, 0.0, 0.0]],
        )
        self.waypoints_units = [(float(w[0]), float(w[1]), float(w[2])) for w in raw_wps]
        self.waypoints_m = [
            (w[0] * self.unit_scale_m, w[1] * self.unit_scale_m, wrap_angle(w[2]))
            for w in self.waypoints_units
        ]

        # Current estimated state [x_m, y_m, theta_rad]
        w0 = self.waypoints_m[0]
        self.pose = [w0[0], w0[1], w0[2]]
        self.pose_std = [0.015, 0.015, 0.03]
        self.visible_landmarks = 0
        self.secs_since_visual = 0.0
        self.received_ekf_pose = False

        # Distance & trajectory accumulators (Report 1g, 1h, 1i)
        self.total_translation_m = 0.0
        self.total_rotation_rad = 0.0
        self.run_start_time = None
        self.last_traj_log_time = 0.0
        self.trajectory_samples = []
        self.waypoint_reports = []

        # ROS Publishers & Subscribers
        self.motor_pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.pose_sub = self.create_subscription(
            Float32MultiArray, '/robot_pose', self.pose_callback, 10
        )

    def pose_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.pose[0] = float(msg.data[0])
            self.pose[1] = float(msg.data[1])
            self.pose[2] = wrap_angle(float(msg.data[2]))
            self.received_ekf_pose = True
        if len(msg.data) >= 6:
            self.pose_std = [float(msg.data[3]), float(msg.data[4]), float(msg.data[5])]
        if len(msg.data) >= 8:
            self.visible_landmarks = int(round(msg.data[6]))
            self.secs_since_visual = float(msg.data[7])

    def send_motor(self, L: float, R: float):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.motor_pub.publish(msg)

    def maybe_log_trajectory(self, force: bool = False):
        if self.run_start_time is None:
            return
        now = time.monotonic()
        if force or (now - self.last_traj_log_time) >= self.log_period_s:
            self.last_traj_log_time = now
            t_rel = now - self.run_start_time
            x_m, y_m, th = self.pose
            self.trajectory_samples.append({
                't_s': round(t_rel, 3),
                'x_m': round(x_m, 4),
                'y_m': round(y_m, 4),
                'theta_rad': round(th, 4),
                'x_unit': round(x_m / self.unit_scale_m, 4),
                'y_unit': round(y_m / self.unit_scale_m, 4),
                'std_x_m': round(self.pose_std[0], 4),
                'std_y_m': round(self.pose_std[1], 4),
                'std_theta_rad': round(self.pose_std[2], 4),
                'visible_landmarks': self.visible_landmarks,
            })

    def wait(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            self.maybe_log_trajectory()

    def execute_turn(self, angle_rad: float, reason: str = ''):
        if abs(angle_rad) < MIN_ANGLE_RAD:
            return
        command, duration = turn_command(angle_rad)
        self.total_rotation_rad += abs(angle_rad)
        self.get_logger().info(
            f'  [TURN {reason}] {math.degrees(angle_rad):+.1f} deg '
            f'(L={command[0]:+.2f}, R={command[1]:+.2f}, {duration:.2f}s)'
        )
        if self.dry_run:
            self.pose[2] = wrap_angle(self.pose[2] + angle_rad)
            self.maybe_log_trajectory(force=True)
            return

        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.send_motor(command[0], command[1])
            self.wait(min(PUBLISH_PERIOD_S, max(0.0, end - time.monotonic())))
        self.send_motor(0.0, 0.0)
        if not self.received_ekf_pose:
            self.pose[2] = wrap_angle(self.pose[2] + angle_rad)
        self.wait(self.settle_time_s)

    def execute_drive(self, distance_m: float, reason: str = ''):
        if abs(distance_m) < MIN_DISTANCE_M:
            return
        command, duration = drive_command(distance_m)
        self.total_translation_m += abs(distance_m)
        self.get_logger().info(
            f'  [DRIVE {reason}] {distance_m * 100:.1f} cm ({distance_m / self.unit_scale_m:.2f} units) '
            f'(L={command[0]:+.3f}, R={command[1]:+.3f}, {duration:.2f}s)'
        )
        if self.dry_run:
            self.pose[0] += distance_m * math.cos(self.pose[2])
            self.pose[1] += distance_m * math.sin(self.pose[2])
            self.maybe_log_trajectory(force=True)
            return

        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.send_motor(command[0], command[1])
            self.wait(min(PUBLISH_PERIOD_S, max(0.0, end - time.monotonic())))
        self.send_motor(0.0, 0.0)
        if not self.received_ekf_pose:
            self.pose[0] += distance_m * math.cos(self.pose[2])
            self.pose[1] += distance_m * math.sin(self.pose[2])
        self.wait(self.settle_time_s)

    def align_heading_closed_loop(self, target_heading_rad: float, label: str):
        """Turn to target_heading_rad and apply 1 visual corrective turn if slip occurred."""
        err = wrap_angle(target_heading_rad - self.pose[2])
        if abs(err) >= self.head_tol_rad:
            self.execute_turn(err, reason=f'{label} primary')

        # Visual feedback verification after settle pause
        residual = wrap_angle(target_heading_rad - self.pose[2])
        if abs(residual) >= self.head_tol_rad:
            self.get_logger().info(
                f'  [VISUAL HEADING CORRECTION] residual={math.degrees(residual):+.1f} deg'
            )
            self.execute_turn(residual, reason=f'{label} visual-trim')

    def navigate_to_waypoint(self, wp_idx: int, gx_m: float, gy_m: float, gtheta_rad: float):
        gx_u, gy_u, _ = self.waypoints_units[wp_idx]
        self.get_logger().info(
            f'=== Waypoint {wp_idx}: target=({gx_u:.1f}u, {gy_u:.1f}u, {math.degrees(gtheta_rad):.0f}deg) '
            f'[=({gx_m:.2f}m, {gy_m:.2f}m)] ==='
        )

        # Iterative closed-loop position convergence (splits long drives + corrects residual error)
        corrections_used = 0
        for step_iter in range(6):
            dx = gx_m - self.pose[0]
            dy = gy_m - self.pose[1]
            dist_err = math.hypot(dx, dy)

            if dist_err <= self.pos_tol_m:
                break

            is_substep = dist_err > self.max_step_m
            if not is_substep and step_iter > 0:
                if corrections_used >= self.max_corrections:
                    self.get_logger().info(
                        f'  Reached max_corrections={self.max_corrections} '
                        f'(remaining dist_err={dist_err * 100:.1f}cm)'
                    )
                    break
                corrections_used += 1

            desired_heading = math.atan2(dy, dx)
            self.align_heading_closed_loop(desired_heading, label=f'WP{wp_idx}-aim')

            # Recompute distance along updated pose after turn
            dx = gx_m - self.pose[0]
            dy = gy_m - self.pose[1]
            dist_now = math.hypot(dx, dy)
            step_dist = min(dist_now, self.max_step_m)
            tag = 'sub-step' if dist_now > self.max_step_m else f'iter-{step_iter}'
            self.execute_drive(step_dist, reason=f'WP{wp_idx} {tag}')

        # Final goal heading alignment at waypoint
        self.align_heading_closed_loop(gtheta_rad, label=f'WP{wp_idx}-goal-heading')
        self.record_waypoint_metrics(wp_idx, gx_m, gy_m, gtheta_rad)

    def record_waypoint_metrics(self, wp_idx: int, gx_m: float, gy_m: float, gtheta_rad: float):
        ex_m = self.pose[0] - gx_m
        ey_m = self.pose[1] - gy_m
        trans_err_m = math.hypot(ex_m, ey_m)
        rot_err_rad = wrap_angle(self.pose[2] - gtheta_rad)

        gx_u, gy_u, _ = self.waypoints_units[wp_idx]
        est_x_u = self.pose[0] / self.unit_scale_m
        est_y_u = self.pose[1] / self.unit_scale_m
        trans_err_u = trans_err_m / self.unit_scale_m

        report = {
            'waypoint_index': wp_idx,
            'target_units': [round(gx_u, 3), round(gy_u, 3), round(gtheta_rad, 4)],
            'target_m': [round(gx_m, 3), round(gy_m, 3), round(gtheta_rad, 4)],
            'estimated_units': [round(est_x_u, 4), round(est_y_u, 4), round(self.pose[2], 4)],
            'estimated_m': [round(self.pose[0], 4), round(self.pose[1], 4), round(self.pose[2], 4)],
            'translation_error_cm': round(trans_err_m * 100.0, 2),
            'translation_error_units': round(trans_err_u, 4),
            'rotation_error_deg': round(math.degrees(rot_err_rad), 2),
            'rotation_error_rad': round(rot_err_rad, 4),
            'ekf_std_cm': [round(self.pose_std[0] * 100.0, 2), round(self.pose_std[1] * 100.0, 2)],
            'ekf_std_deg': round(math.degrees(self.pose_std[2]), 2),
            'visible_landmarks': self.visible_landmarks,
        }
        self.waypoint_reports.append(report)
        self.get_logger().info(
            f'  [WP{wp_idx} ARRIVAL] Est=({self.pose[0]:.3f}m, {self.pose[1]:.3f}m, '
            f'{math.degrees(self.pose[2]):+.1f}deg) | '
            f'Err_trans={trans_err_m * 100:.2f}cm ({trans_err_u:.3f}u), '
            f'Err_rot={math.degrees(rot_err_rad):+.2f}deg'
        )

    def save_logs_and_summary(self):
        os.makedirs(self.log_dir, exist_ok=True)
        csv_path = os.path.join(self.log_dir, 'hw2_trajectory.csv')
        json_path = os.path.join(self.log_dir, 'hw2_summary.json')

        if self.trajectory_samples:
            fieldnames = list(self.trajectory_samples[0].keys())
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.trajectory_samples)

        total_trans_u = self.total_translation_m / self.unit_scale_m
        summary = {
            'unit_scale_m': self.unit_scale_m,
            'total_translation_m': round(self.total_translation_m, 4),
            'total_translation_units': round(total_trans_u, 4),
            'total_rotation_rad': round(self.total_rotation_rad, 4),
            'total_rotation_deg': round(math.degrees(self.total_rotation_rad), 2),
            'combined_distance_units_plus_rad': round(total_trans_u + self.total_rotation_rad, 4),
            'waypoint_errors': self.waypoint_reports,
        }
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info('================================================')
        self.get_logger().info(
            f'Total Translation: {self.total_translation_m:.3f} m ({total_trans_u:.3f} units)'
        )
        self.get_logger().info(
            f'Total Rotation   : {self.total_rotation_rad:.3f} rad '
            f'({math.degrees(self.total_rotation_rad):.1f} deg)'
        )
        self.get_logger().info(f'Saved trajectory log : {csv_path}')
        self.get_logger().info(f'Saved metrics summary: {json_path}')
        self.get_logger().info('================================================')

    def run(self):
        self.get_logger().info(
            f'ClosedLoopFollower ready: {len(self.waypoints_m)} waypoints, '
            f'1 unit = {self.unit_scale_m:.2f} m'
        )
        if not self.dry_run:
            self.get_logger().info('Waiting for motor_control and initial EKF pose...')
            wait_start = time.monotonic()
            while (
                self.motor_pub.get_subscription_count() == 0
                and (time.monotonic() - wait_start) < 10.0
                and rclpy.ok()
            ):
                rclpy.spin_once(self, timeout_sec=0.1)

            for i in range(3, 0, -1):
                self.get_logger().info(f'Starting closed-loop navigation in {i}...')
                self.wait(1.0)

        self.run_start_time = time.monotonic()
        self.maybe_log_trajectory(force=True)

        # Record start pose W0 = (0, 0, 0)
        w0 = self.waypoints_m[0]
        self.record_waypoint_metrics(0, w0[0], w0[1], w0[2])

        # Traverse W1, W2, W3 in closed loop
        for idx in range(1, len(self.waypoints_m)):
            gx_m, gy_m, gth = self.waypoints_m[idx]
            self.navigate_to_waypoint(idx, gx_m, gy_m, gth)

        self.maybe_log_trajectory(force=True)
        self.save_logs_and_summary()


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopFollowerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_motor(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
