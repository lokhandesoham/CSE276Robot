"""Interactive ROS 2 chessboard camera calibration node for Rubik Pi.

Subscribes to `/camera/image_raw`, detects chessboard inner corners, publishes a live
annotated preview on `/calibration_debug_image/compressed` (for Foxglove Studio),
collects diverse views, runs `cv2.calibrateCamera`, and saves the OpenCV `%YAML:1.0`
`!!opencv-matrix` format required by `robot_vision_camera`.

Usage:
    ros2 run robot_control calibrate_camera
    ros2 run robot_control calibrate_camera --ros-args -p cols:=9 -p rows:=6 -p square_size:=0.025
"""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


DEFAULT_OUTPUT_YAML = os.path.expanduser(
    '~/ros2_ws/src/rubikpi_ros2/robot_vision/config/camera_parameter.yaml'
)
FALLBACK_LOCAL_YAML = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), '..', '..', 'robot_vision', 'config', 'camera_parameter.yaml'
    )
)


def format_opencv_matrix_yaml(name, mat):
    """Format a 2D numpy array as an OpenCV FileStorage !!opencv-matrix YAML block."""
    rows, cols = mat.shape
    flat = [f'{float(v):.6f}' for v in mat.flatten()]
    row_strs = []
    for r in range(rows):
        row_vals = ', '.join(flat[r * cols:(r + 1) * cols])
        row_strs.append(row_vals)
    joined_rows = (',\n         ').join(row_strs)
    return (
        f'{name}: !!opencv-matrix\n'
        f'  rows: {rows}\n'
        f'  cols: {cols}\n'
        f'  dt: d\n'
        f'  data: [{joined_rows}]\n'
    )


def write_camera_parameter_yaml(path, width, height, K, D):
    """Write camera_parameter.yaml in the exact format parsed by robot_vision_camera.h."""
    D_flat = np.array(D, dtype=np.float64).reshape(1, -1)
    if D_flat.shape[1] > 5:
        D_flat = D_flat[:, :5]
    elif D_flat.shape[1] < 5:
        padded = np.zeros((1, 5), dtype=np.float64)
        padded[0, :D_flat.shape[1]] = D_flat[0]
        D_flat = padded

    R = np.eye(3, dtype=np.float64)
    P = np.zeros((3, 4), dtype=np.float64)
    P[:3, :3] = K

    content = (
        '%YAML:1.0\n'
        '---\n'
        f'image_width: {int(width)}\n'
        f'image_height: {int(height)}\n'
        'camera_name: camera\n\n'
        + format_opencv_matrix_yaml('camera_matrix', K)
        + '\ndistortion_model: plumb_bob\n'
        + format_opencv_matrix_yaml('distortion_coefficients', D_flat)
        + '\n'
        + format_opencv_matrix_yaml('rectification_matrix', R)
        + '\n'
        + format_opencv_matrix_yaml('projection_matrix', P)
    )
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content)


class ChessboardCalibrationNode(Node):
    def __init__(self):
        super().__init__('calibrate_camera_node')

        self.cols = int(self.declare_parameter('cols', 9).value)
        self.rows = int(self.declare_parameter('rows', 6).value)
        self.square_size = float(self.declare_parameter('square_size', 0.025).value)
        self.target_samples = int(self.declare_parameter('target_samples', 20).value)
        self.min_interval_s = float(self.declare_parameter('min_interval_s', 1.2).value)
        self.min_shift_px = float(self.declare_parameter('min_shift_px', 35.0).value)
        self.image_topic = str(self.declare_parameter('image_topic', '/camera/image_raw').value)

        default_out = DEFAULT_OUTPUT_YAML if os.path.exists(
            os.path.dirname(DEFAULT_OUTPUT_YAML)
        ) else FALLBACK_LOCAL_YAML
        self.output_yaml = str(self.declare_parameter('output_yaml', default_out).value)

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, '/calibration_debug_image', 10)
        self.debug_compressed_pub = self.create_publisher(
            CompressedImage, '/calibration_debug_image/compressed', 10
        )

        self.obj_points = []
        self.img_points = []
        self.sample_descriptors = []  # (cx, cy, span) of saved boards
        self.prev_corners_center = None
        self.last_capture_time = 0.0
        self.image_size = None
        self.calibrated = False

        self._rebuild_object_grid()

        self.get_logger().info(
            f'Chessboard Calibrator started: looking for {self.cols}x{self.rows} INNER corners '
            f'(square_size={self.square_size * 1000:.1f} mm) on {self.image_topic}'
        )
        self.get_logger().info(
            f'Move the chessboard slowly across the camera view until {self.target_samples} '
            'samples are captured (or press Ctrl+C after >=8 samples to calibrate early).'
        )
        self.get_logger().info(f'Output YAML path: {self.output_yaml}')

    def _rebuild_object_grid(self):
        objp = np.zeros((self.rows * self.cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= self.square_size
        self.objp_template = objp

    def _detect_corners(self, gray):
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), flags)
        if found:
            return found, corners

        # If no samples collected yet, check if user's printout has a different inner-corner count
        if len(self.img_points) == 0:
            for c, r in [(8, 6), (7, 5), (7, 6), (9, 7), (8, 5), (6, 4)]:
                if (c, r) == (self.cols, self.rows):
                    continue
                ok, alt_corners = cv2.findChessboardCorners(gray, (c, r), flags)
                if ok:
                    self.get_logger().info(
                        f'Auto-adjusted chessboard inner corners from {self.cols}x{self.rows} '
                        f'to {c}x{r}!'
                    )
                    self.cols, self.rows = c, r
                    self._rebuild_object_grid()
                    return True, alt_corners
        return False, None

    def image_callback(self, msg: Image):
        if self.calibrated:
            return

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
        except Exception as e:
            self.get_logger().error(f'Failed to decode image buffer: {e}')
            return

        self.image_size = (msg.width, msg.height)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = self._detect_corners(gray)
        status_text = f'Samples: {len(self.img_points)}/{self.target_samples} ({self.cols}x{self.rows})'
        status_color = (0, 200, 255)

        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, (self.cols, self.rows), corners_sub, found)

            pts = corners_sub.reshape(-1, 2)
            cx, cy = float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
            span = float(np.hypot(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))

            # Check steadiness (low motion between consecutive frames)
            steady = True
            if self.prev_corners_center is not None:
                motion_px = math.hypot(cx - self.prev_corners_center[0], cy - self.prev_corners_center[1])
                steady = motion_px < 15.0
            self.prev_corners_center = (cx, cy)

            # Check diversity compared to already saved samples
            diverse = True
            for scx, scy, sspan in self.sample_descriptors:
                dist = math.hypot(cx - scx, cy - scy) + 0.5 * abs(span - sspan)
                if dist < self.min_shift_px:
                    diverse = False
                    break

            now = time.monotonic()
            if steady and diverse and (now - self.last_capture_time) >= self.min_interval_s:
                self.obj_points.append(self.objp_template.copy())
                self.img_points.append(corners_sub)
                self.sample_descriptors.append((cx, cy, span))
                self.last_capture_time = now
                status_color = (0, 255, 0)
                status_text = f'CAPTURED {len(self.img_points)}/{self.target_samples}!'
                self.get_logger().info(
                    f'Captured sample {len(self.img_points)}/{self.target_samples} '
                    f'at center=({cx:.0f}, {cy:.0f}), span={span:.0f}px'
                )
            elif not diverse:
                status_text += ' - Move board to new angle/pos'
            elif not steady:
                status_text += ' - Hold steady'
        else:
            self.prev_corners_center = None
            status_text += ' - No board detected'

        cv2.putText(
            frame, status_text, (25, 45),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2, cv2.LINE_AA
        )
        self._publish_debug(frame, msg.header)

        if len(self.img_points) >= self.target_samples:
            self.run_calibration()

    def _publish_debug(self, frame, header):
        raw_msg = Image()
        raw_msg.header = header
        raw_msg.height, raw_msg.width = frame.shape[:2]
        raw_msg.encoding = 'rgb8'
        raw_msg.step = frame.shape[1] * 3
        raw_msg.data = frame.tobytes()
        self.debug_pub.publish(raw_msg)

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = 'jpeg'
            comp_msg.data = buf.tobytes()
            self.debug_compressed_pub.publish(comp_msg)

    def run_calibration(self):
        if self.calibrated or len(self.img_points) < 5 or self.image_size is None:
            return False

        self.calibrated = True
        w, h = self.image_size
        self.get_logger().info(
            f'Running cv2.calibrateCamera on {len(self.img_points)} samples ({w}x{h})...'
        )
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, (w, h), None, None
        )

        write_camera_parameter_yaml(self.output_yaml, w, h, K, D)
        if os.path.abspath(self.output_yaml) != os.path.abspath(FALLBACK_LOCAL_YAML):
            try:
                write_camera_parameter_yaml(FALLBACK_LOCAL_YAML, w, h, K, D)
            except Exception:
                pass

        self.get_logger().info('================================================')
        self.get_logger().info(f'Calibration complete! RMS Reprojection Error: {rms:.4f} px')
        self.get_logger().info(
            f'fx={K[0, 0]:.2f}, fy={K[1, 1]:.2f}, cx={K[0, 2]:.2f}, cy={K[1, 2]:.2f}'
        )
        self.get_logger().info(f'D = {np.round(D.flatten(), 5).tolist()}')
        self.get_logger().info(f'Saved OpenCV YAML to: {self.output_yaml}')
        self.get_logger().info('================================================')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardCalibrationNode()
    try:
        while rclpy.ok() and not node.calibrated:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        if not node.calibrated and len(node.img_points) >= 8:
            node.get_logger().info('Interrupted with >=8 samples; running calibration now...')
            node.run_calibration()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
