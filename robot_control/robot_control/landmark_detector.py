"""Custom Landmark Detector Node for CSE 276A HW2 (+20% Extra Credit, No AprilTags).

Loads a single Ultralytics YOLO model with two selectable configuration modes:
  1. `model_mode: "yolo_world"` -> Open-vocabulary `yolov8s-worldv2.pt` with custom prompts
  2. `model_mode: "yolov8s"`    -> Standard COCO `yolov8s.pt` mapped to landmark classes

For each detected landmark, undistorts bounding-box keypoints using the calibrated camera
intrinsics (K, D) from `camera_parameter.yaml` and estimates relative range `r` (meters)
and bearing `alpha` (radians, +CCW in vehicle frame).
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float32MultiArray
import yaml


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


def resolve_default_camera_yaml():
    candidates = [
        os.path.expanduser('~/ros2_ws/src/rubikpi_ros2/robot_vision/config/camera_parameter.yaml'),
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), '..', '..', 'robot_vision', 'config', 'camera_parameter.yaml'
            )
        ),
    ]
    try:
        share_dir = get_package_share_directory('robot_vision_camera')
        candidates.append(os.path.join(share_dir, 'config', 'camera_parameter.yaml'))
    except Exception:
        pass

    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


def load_camera_intrinsics(yaml_path):
    """Load camera_matrix (K) and distortion_coefficients (D) from OpenCV FileStorage YAML."""
    if not os.path.exists(yaml_path):
        return None, None

    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None, None

    K_node = fs.getNode('camera_matrix')
    D_node = fs.getNode('distortion_coefficients')
    K = K_node.mat() if not K_node.empty() else None
    D = D_node.mat() if not D_node.empty() else None
    fs.release()

    if K is not None and D is not None:
        return np.array(K, dtype=np.float64).reshape(3, 3), np.array(D, dtype=np.float64).flatten()
    return None, None


class LandmarkDetectorNode(Node):
    def __init__(self):
        super().__init__('landmark_detector_node')

        config_path = self.declare_parameter('config_file', resolve_default_config_path()).value
        cam_yaml_path = self.declare_parameter('camera_yaml', resolve_default_camera_yaml()).value
        override_mode = self.declare_parameter('model_mode', '').value
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value

        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        det_cfg = self.cfg.get('detector', {})
        self.model_mode = override_mode if override_mode else det_cfg.get('model_mode', 'yolo_world')
        self.conf_threshold = float(det_cfg.get('conf_threshold', 0.35))
        self.iou_threshold = float(det_cfg.get('iou_threshold', 0.45))
        self.imgsz = int(det_cfg.get('imgsz', 640))
        self.swap_rb = bool(det_cfg.get('swap_rb', False))
        self.min_box_height_px = float(det_cfg.get('min_box_height_px', 18.0))
        self.border_margin_px = int(det_cfg.get('border_margin_px', 10))

        # Parse known landmarks
        self.landmarks = {}
        self.prompt_list = []
        self.prompt_idx_to_id = {}
        self.coco_name_to_id = {}

        for lm in self.cfg.get('landmarks', []):
            lm_id = int(lm['id'])
            self.landmarks[lm_id] = {
                'id': lm_id,
                'name': str(lm['name']),
                'height_m': float(lm['height_m']),
                'width_m': float(lm.get('width_m', lm['height_m'])),
                'scale_factor': float(lm.get('scale_factor', 1.0)),
                'prompt': str(lm.get('prompt', lm['name'])),
                'coco_classes': [c.lower() for c in lm.get('coco_classes', [])],
            }
            prompt_idx = len(self.prompt_list)
            self.prompt_list.append(self.landmarks[lm_id]['prompt'])
            self.prompt_idx_to_id[prompt_idx] = lm_id
            for coco_cls in self.landmarks[lm_id]['coco_classes']:
                self.coco_name_to_id[coco_cls] = lm_id

        # Load camera intrinsics
        self.K, self.D = load_camera_intrinsics(cam_yaml_path)
        if self.K is not None:
            self.get_logger().info(
                f'Loaded camera intrinsics from {cam_yaml_path}: '
                f'fx={self.K[0, 0]:.1f}, fy={self.K[1, 1]:.1f}, '
                f'cx={self.K[0, 2]:.1f}, cy={self.K[1, 2]:.1f}'
            )
        else:
            self.get_logger().warn(
                f'Could not load {cam_yaml_path}; waiting for /camera/camera_info...'
            )

        # Load the single YOLO model according to model_mode ("yolo_world" or "yolov8s")
        self.model = None
        self._init_yolo_model(det_cfg)

        # ROS Publishers & Subscribers
        self.det_pub = self.create_publisher(Float32MultiArray, '/landmark_detections', 10)
        self.debug_pub = self.create_publisher(Image, '/landmark_debug_image', 10)
        self.debug_comp_pub = self.create_publisher(
            CompressedImage, '/landmark_debug_image/compressed', 10
        )

        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )
        self.img_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

    def _init_yolo_model(self, det_cfg):
        from ultralytics import YOLO

        if self.model_mode == 'yolo_world':
            weights = det_cfg.get('yolo_world_weights', 'yolov8s-worldv2.pt')
            self.get_logger().info(
                f'Loading YOLO-World model ({weights}) with prompts: {self.prompt_list}'
            )
            self.model = YOLO(weights)
            self.model.set_classes(self.prompt_list)
        elif self.model_mode == 'yolov8s':
            weights = det_cfg.get('yolov8s_weights', 'yolov8s.pt')
            self.get_logger().info(
                f'Loading standard YOLOv8s COCO model ({weights}) mapped to: '
                f'{list(self.coco_name_to_id.keys())}'
            )
            self.model = YOLO(weights)
        else:
            raise ValueError(
                f'Unknown model_mode "{self.model_mode}". Choose "yolo_world" or "yolov8s".'
            )

    def camera_info_callback(self, msg: CameraInfo):
        if self.K is None and len(msg.k) == 9 and msg.k[0] > 0:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).flatten()
            self.get_logger().info(
                f'Received camera_info: fx={self.K[0, 0]:.1f}, fy={self.K[1, 1]:.1f}'
            )

    def undistort_keypoints(self, u_c, v_c, v_top, v_bot):
        """Undistort bounding-box center, top, and bottom using K and D."""
        if self.K is None or self.D is None or np.all(self.D == 0):
            return u_c, v_c, v_top, v_bot

        pts = np.array(
            [[[u_c, v_c]], [[u_c, v_top]], [[u_c, v_bot]]],
            dtype=np.float64,
        )
        undist = cv2.undistortPoints(pts, self.K, self.D, P=self.K)
        u_c_u, v_c_u = float(undist[0, 0, 0]), float(undist[0, 0, 1])
        v_top_u = float(undist[1, 0, 1])
        v_bot_u = float(undist[2, 0, 1])
        return u_c_u, v_c_u, v_top_u, v_bot_u

    def estimate_range_bearing(self, lm, bbox, img_h):
        """Compute range r (meters) and bearing alpha (radians, +CCW) from 2D bbox."""
        x1, y1, x2, y2 = bbox
        u_c = 0.5 * (x1 + x2)
        v_c = 0.5 * (y1 + y2)

        is_truncated = (
            y1 <= self.border_margin_px or y2 >= (img_h - self.border_margin_px)
        )

        fx = float(self.K[0, 0]) if self.K is not None else 491.0
        fy = float(self.K[1, 1]) if self.K is not None else 492.1
        cx = float(self.K[0, 2]) if self.K is not None else 639.4

        u_c_u, _, v_top_u, v_bot_u = self.undistort_keypoints(u_c, v_c, y1, y2)
        delta_v = max(1.0, abs(v_bot_u - v_top_u))

        # Horizontal bearing angle (+CCW / left in vehicle frame)
        bearing_rad = -math.atan2(u_c_u - cx, fx)

        # Optical depth Z and Euclidean range r
        z_m = lm['scale_factor'] * (fy * lm['height_m']) / delta_v
        range_m = z_m / max(0.25, math.cos(bearing_rad))
        return range_m, bearing_rad, is_truncated

    def image_callback(self, msg: Image):
        if self.model is None:
            return

        t0 = time.monotonic()
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
        except Exception as e:
            self.get_logger().error(f'Image reshape error: {e}')
            return

        if self.swap_rb:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.imgsz,
            verbose=False,
        )

        # Keep highest-confidence detection per landmark ID
        best_per_lm = {}
        if results and len(results) > 0:
            res = results[0]
            names = res.names
            boxes = res.boxes
            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clss = boxes.cls.cpu().numpy().astype(int)

                for box, conf, cls_idx in zip(xyxy, confs, clss):
                    if (box[3] - box[1]) < self.min_box_height_px:
                        continue

                    if self.model_mode == 'yolo_world':
                        lm_id = self.prompt_idx_to_id.get(int(cls_idx))
                    else:
                        cls_name = str(names.get(int(cls_idx), '')).lower()
                        lm_id = self.coco_name_to_id.get(cls_name)

                    if lm_id is None:
                        continue

                    if lm_id not in best_per_lm or float(conf) > best_per_lm[lm_id][0]:
                        best_per_lm[lm_id] = (float(conf), box)

        flat_detections = []
        debug_frame = frame.copy()

        for lm_id, (conf, box) in sorted(best_per_lm.items()):
            lm = self.landmarks[lm_id]
            range_m, bearing_rad, is_truncated = self.estimate_range_bearing(
                lm, box, msg.height
            )
            flat_detections.extend([
                float(lm_id),
                float(range_m),
                float(bearing_rad),
                float(conf),
                1.0 if is_truncated else 0.0,
            ])

            # Draw bounding box & pose telemetry for Foxglove / Video
            x1, y1, x2, y2 = [int(v) for v in box]
            color = (0, 165, 255) if is_truncated else (0, 255, 0)
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
            label = (
                f'ID{lm_id}:{lm["name"]} ({conf:.2f}) '
                f'r={range_m:.2f}m a={math.degrees(bearing_rad):+.1f}d'
            )
            cv2.putText(
                debug_frame, label, (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA
            )

        det_msg = Float32MultiArray()
        det_msg.data = flat_detections
        self.det_pub.publish(det_msg)

        fps = 1.0 / max(1e-3, time.monotonic() - t0)
        cv2.putText(
            debug_frame,
            f'Mode: {self.model_mode} | FPS: {fps:.1f} | Landmarks: {len(best_per_lm)}',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        self._publish_debug(debug_frame, msg.header)

    def _publish_debug(self, frame, header):
        if self.debug_pub.get_subscription_count() > 0:
            raw_msg = Image()
            raw_msg.header = header
            raw_msg.height, raw_msg.width = frame.shape[:2]
            raw_msg.encoding = 'rgb8'
            raw_msg.step = frame.shape[1] * 3
            raw_msg.data = frame.tobytes()
            self.debug_pub.publish(raw_msg)

        if self.debug_comp_pub.get_subscription_count() > 0:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                comp_msg = CompressedImage()
                comp_msg.header = header
                comp_msg.format = 'jpeg'
                comp_msg.data = buf.tobytes()
                self.debug_comp_pub.publish(comp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDetectorNode()
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
