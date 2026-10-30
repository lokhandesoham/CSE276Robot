"""Map-Based Extended Kalman Filter (EKF) Pose Estimator for CSE 276A HW2 (No SLAM).

Fuses the calibrated HW1 unicycle motion model (`/motor_commands`) with range-bearing
visual measurements (`/landmark_detections`) against known landmark coordinates from
`hw2_config.yaml`.

Publishes:
  - `/robot_pose` (std_msgs/Float32MultiArray):
      [x_m, y_m, theta_rad, std_x, std_y, std_theta, visible_landmarks, secs_since_visual]
  - `/robot_pose_stamped` (geometry_msgs/PoseStamped): for Foxglove 3D visualization.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import yaml

from robot_control.calibration import (
    DRIVE_SPEED,
    TURN_DELAY_LEFT,
    TURN_DELAY_RIGHT,
    TURN_RATE_LEFT,
    TURN_RATE_RIGHT,
)


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


class RangeBearingEKF:
    """3-DoF [x, y, theta]^T Extended Kalman Filter with known landmark map."""

    def __init__(self, init_pose=(0.0, 0.0, 0.0), ekf_cfg=None):
        cfg = ekf_cfg or {}
        self.d_cam = float(cfg.get('camera_offset_x_m', 0.08))

        init_std_xy = float(cfg.get('init_std_xy_m', 0.015))
        init_std_th = float(cfg.get('init_std_theta_rad', 0.03))

        self.q_drive_xy = float(cfg.get('q_drive_xy_per_m', 0.03))
        self.q_drive_th = float(cfg.get('q_drive_theta_per_m', 0.07))
        self.q_turn_xy = float(cfg.get('q_turn_xy_per_rad', 0.015))
        self.q_turn_th = float(cfg.get('q_turn_theta_per_rad', 0.08))

        self.r_range_base = float(cfg.get('r_range_base_m', 0.035))
        self.r_range_per_m = float(cfg.get('r_range_per_m', 0.06))
        self.r_bearing = float(cfg.get('r_bearing_rad', 0.030))
        self.mahalanobis_gate = float(cfg.get('mahalanobis_gate', 9.21))

        self.x = np.array(
            [float(init_pose[0]), float(init_pose[1]), wrap_angle(float(init_pose[2]))],
            dtype=np.float64,
        )
        self.P = np.diag([init_std_xy ** 2, init_std_xy ** 2, init_std_th ** 2])

    def predict(self, v: float, omega: float, dt: float):
        """Propagate state and covariance over timestep dt using unicycle kinematics."""
        if dt <= 0.0 or (abs(v) < 1e-6 and abs(omega) < 1e-6):
            return

        theta = float(self.x[2])
        ds = v * dt
        dtheta = omega * dt

        # State prediction
        self.x[0] += ds * math.cos(theta)
        self.x[1] += ds * math.sin(theta)
        self.x[2] = wrap_angle(theta + dtheta)

        # Jacobian F = df/dx
        F = np.array([
            [1.0, 0.0, -ds * math.sin(theta)],
            [0.0, 1.0,  ds * math.cos(theta)],
            [0.0, 0.0,  1.0],
        ], dtype=np.float64)

        # Motion-scaled process noise covariance Q
        dist_abs = abs(ds)
        turn_abs = abs(dtheta)
        var_xy = (self.q_drive_xy * dist_abs) ** 2 + (self.q_turn_xy * turn_abs) ** 2 + 1e-7
        var_th = (self.q_drive_th * dist_abs) ** 2 + (self.q_turn_th * turn_abs) ** 2 + 1e-7
        Q = np.diag([var_xy, var_xy, var_th])

        self.P = F @ self.P @ F.T + Q

    def update_landmark(
        self,
        lm_x: float,
        lm_y: float,
        meas_r: float,
        meas_alpha: float,
        conf: float = 1.0,
        is_truncated: bool = False,
    ) -> bool:
        """Perform EKF range-bearing measurement update for a single known landmark."""
        rx, ry, rtheta = float(self.x[0]), float(self.x[1]), float(self.x[2])

        # Camera optical center in world coordinates
        cam_x = rx + self.d_cam * math.cos(rtheta)
        cam_y = ry + self.d_cam * math.sin(rtheta)

        dx = lm_x - cam_x
        dy = lm_y - cam_y
        q = dx * dx + dy * dy
        pred_r = math.sqrt(max(1e-6, q))
        pred_alpha = wrap_angle(math.atan2(dy, dx) - rtheta)

        # Innovation y = z - h(x)
        y = np.array([
            meas_r - pred_r,
            wrap_angle(meas_alpha - pred_alpha),
        ], dtype=np.float64)

        # Measurement Jacobian H = dh/dx (including camera lever arm d_cam)
        dr_dtheta = ((dx * math.sin(rtheta) - dy * math.cos(rtheta)) / pred_r) * self.d_cam
        da_dtheta = -1.0 - ((dx * math.cos(rtheta) + dy * math.sin(rtheta)) / max(1e-6, q)) * self.d_cam

        H = np.array([
            [-dx / pred_r, -dy / pred_r, dr_dtheta],
            [ dy / q,      -dx / q,      da_dtheta],
        ], dtype=np.float64)

        # Measurement noise covariance R (scale slightly with confidence & truncation)
        conf_scale = 1.0 / max(0.35, min(1.0, conf))
        sigma_r = (self.r_range_base + self.r_range_per_m * meas_r) * conf_scale
        if is_truncated:
            # Bbox touches vertical image edge -> ignore range, keep horizontal bearing!
            sigma_r = 10.0
        sigma_a = self.r_bearing * conf_scale
        R = np.diag([sigma_r ** 2, sigma_a ** 2])

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

        # Mahalanobis distance outlier gate
        maha_sq = float(y.T @ S_inv @ y)
        if maha_sq > self.mahalanobis_gate:
            return False

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(float(self.x[2]))

        # Joseph form covariance update for numerical stability
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        config_path = self.declare_parameter('config_file', resolve_default_config_path()).value
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.unit_scale_m = float(
            self.declare_parameter('unit_scale_m', self.cfg.get('unit_scale_m', 0.5)).value
        )

        # Load starting waypoint as initial EKF state
        waypoints_u = self.cfg.get('waypoints_units', [[0.0, 0.0, 0.0]])
        w0 = waypoints_u[0]
        init_pose = (w0[0] * self.unit_scale_m, w0[1] * self.unit_scale_m, w0[2])

        self.ekf = RangeBearingEKF(init_pose=init_pose, ekf_cfg=self.cfg.get('ekf', {}))

        # Load known landmark world positions in meters
        self.landmark_map = {}
        for lm in self.cfg.get('landmarks', []):
            lm_id = int(lm['id'])
            if 'x_m' in lm and 'y_m' in lm:
                lx, ly = float(lm['x_m']), float(lm['y_m'])
            else:
                lx = float(lm['x_unit']) * self.unit_scale_m
                ly = float(lm['y_unit']) * self.unit_scale_m
            self.landmark_map[lm_id] = (lx, ly, str(lm.get('name', f'lm_{lm_id}')))
            self.get_logger().info(
                f'Landmark {lm_id} ({self.landmark_map[lm_id][2]}): '
                f'world=({lx:.2f}m, {ly:.2f}m)'
            )

        # Track motor command state for dead-reckoning prediction
        self.current_cmd = (0.0, 0.0)
        self.cmd_mode = 'stop'  # 'stop', 'drive_fwd', 'drive_rev', 'turn_left', 'turn_right'
        self.cmd_start_time = time.monotonic()
        self.last_cmd_msg_time = time.monotonic()
        self.last_predict_time = time.monotonic()
        self.last_visual_update_time = time.monotonic()
        self.last_visible_count = 0

        # Subscribers & Publishers
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 'motor_commands', self.motor_cmd_callback, 10
        )
        self.det_sub = self.create_subscription(
            Float32MultiArray, '/landmark_detections', self.detections_callback, 10
        )

        self.pose_pub = self.create_publisher(Float32MultiArray, '/robot_pose', 10)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, '/robot_pose_stamped', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz prediction & publish
        self.get_logger().info(
            f'PoseEstimatorNode initialized at ({init_pose[0]:.2f}m, {init_pose[1]:.2f}m, '
            f'{math.degrees(init_pose[2]):.1f}deg) with unit_scale={self.unit_scale_m}m'
        )

    def _classify_cmd(self, L: float, R: float) -> str:
        if abs(L) < 1e-3 and abs(R) < 1e-3:
            return 'stop'
        if L > 0.0 and R > 0.0:
            return 'drive_fwd'
        if L < 0.0 and R < 0.0:
            return 'drive_rev'
        if L < 0.0 and R > 0.0:
            return 'turn_left'
        if L > 0.0 and R < 0.0:
            return 'turn_right'
        return 'custom'

    def motor_cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        L, R = float(msg.data[0]), float(msg.data[1])
        now = time.monotonic()
        new_mode = self._classify_cmd(L, R)
        if new_mode != self.cmd_mode:
            self.cmd_mode = new_mode
            self.cmd_start_time = now
        self.current_cmd = (L, R)
        self.last_cmd_msg_time = now

    def _current_velocity_model(self, now: float):
        """Convert active motor command into (v, omega) using HW1 calibration."""
        # Safety timeout matches motor_control.py (0.15s)
        if (now - self.last_cmd_msg_time) > 0.15 or self.cmd_mode == 'stop':
            self.cmd_mode = 'stop'
            return 0.0, 0.0

        elapsed = now - self.cmd_start_time
        if self.cmd_mode == 'drive_fwd':
            return DRIVE_SPEED, 0.0
        if self.cmd_mode == 'drive_rev':
            return -DRIVE_SPEED, 0.0
        if self.cmd_mode == 'turn_left':
            if elapsed < TURN_DELAY_LEFT:
                return 0.0, 0.0
            return 0.0, TURN_RATE_LEFT
        if self.cmd_mode == 'turn_right':
            if elapsed < TURN_DELAY_RIGHT:
                return 0.0, 0.0
            return 0.0, -TURN_RATE_RIGHT
        return 0.0, 0.0

    def detections_callback(self, msg: Float32MultiArray):
        data = msg.data
        n = len(data) // 5
        accepted = 0
        for i in range(n):
            lm_id = int(round(data[5 * i + 0]))
            meas_r = float(data[5 * i + 1])
            meas_a = float(data[5 * i + 2])
            conf = float(data[5 * i + 3])
            is_trunc = bool(data[5 * i + 4] > 0.5)

            if lm_id not in self.landmark_map:
                continue

            lx, ly, _ = self.landmark_map[lm_id]
            ok = self.ekf.update_landmark(
                lx, ly, meas_r, meas_a, conf=conf, is_truncated=is_trunc
            )
            if ok:
                accepted += 1

        self.last_visible_count = accepted
        if accepted > 0:
            self.last_visual_update_time = time.monotonic()

    def timer_callback(self):
        now = time.monotonic()
        dt = now - self.last_predict_time
        self.last_predict_time = now

        v, omega = self._current_velocity_model(now)
        self.ekf.predict(v, omega, dt)

        x, y, theta = float(self.ekf.x[0]), float(self.ekf.x[1]), float(self.ekf.x[2])
        std_x = math.sqrt(max(0.0, float(self.ekf.P[0, 0])))
        std_y = math.sqrt(max(0.0, float(self.ekf.P[1, 1])))
        std_th = math.sqrt(max(0.0, float(self.ekf.P[2, 2])))
        secs_since_vis = now - self.last_visual_update_time
        if secs_since_vis > 0.5:
            self.last_visible_count = 0

        msg = Float32MultiArray()
        msg.data = [
            float(x),
            float(y),
            float(theta),
            float(std_x),
            float(std_y),
            float(std_th),
            float(self.last_visible_count),
            float(secs_since_vis),
        ]
        self.pose_pub.publish(msg)

        # Publish PoseStamped for Foxglove 3D visualization
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = math.cos(0.5 * theta)
        ps.pose.orientation.z = math.sin(0.5 * theta)
        self.pose_stamped_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
