"""HW3: EKF-SLAM while driving a path (square or octagon) several times, stop-and-go.

Usage (camera with image_rectify:=true, apriltag_node and motor_control running):
    ros2 run robot_control slam_follower --ros-args -p path_file:=.../path_hw3_square.txt -p loops:=3
    ros2 run robot_control slam_follower --ros-args -p stationary_time:=30.0   # noise test, no motion
    ros2 run robot_control slam_follower --ros-args -p dry_run:=true            # no motors

The robot knows NO landmark positions and not how many there are. The map frame is its start
pose (0, 0, 0). The path is given relative to the start. At every stop:
    1. look     stand still observe_time s, collect AprilTag poses per tag id
    2. update   per tag: median sighting -> EKF update, or augment the state if the id is new
                (ekf_slam.py)
    3. decide   next turn/drive toward the current path point from the SLAM pose estimate;
                turns are made in pieces of at most 46 deg with a look after each piece
    4. move     play it with the HW1 calibrated model (calibration.py), EKF prediction

Output: ~/hw3_runs/slam_<time>.jsonl, one JSON object per line:
    header (parameters, path), motion (kind, value), look (raw frames per tag, statuses,
    pose after the update), loop (end of loop k, with the full state), end (final state).
tools/slam_replay.py re-runs the same EKF on it and makes the plots and the error table.
"""
import json
import math
import os
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control import ekf_slam
from robot_control.calibration import drive_command, turn_command
from robot_control.ekf_slam import EkfSlam, fuse_look, load_path, next_motion
from robot_control.localization import CAMERA_OFFSET, tag_in_robot_frame


PUBLISH_PERIOD_S = 0.05
CAMERA_FRAME = 'camera_frame'
MAX_STEPS = 40          # per path point, in case it never converges

DEFAULT_ROOT = os.path.expanduser('~/ros2_ws/src/rubikpi_ros2')
DEFAULT_PATH = os.path.join(DEFAULT_ROOT, 'path_hw3_square.txt')
LOG_DIR = os.path.expanduser('~/hw3_runs')


class SlamFollowerNode(Node):
    def __init__(self):
        super().__init__('slam_follower_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.camera_frames = 0
        self.apriltag_frames = 0
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_detections, 10)
        self.path_file = self.declare_parameter('path_file', DEFAULT_PATH).value
        self.loops = self.declare_parameter('loops', 3).value
        self.step = self.declare_parameter('step', 0.25).value
        self.settle_time = self.declare_parameter('settle_time', 1.0).value
        self.observe_time = self.declare_parameter('observe_time', 0.8).value
        self.stationary_time = self.declare_parameter('stationary_time', 0.0).value
        self.dry_run = self.declare_parameter('dry_run', False).value

        self.collecting = False
        self.frames = {}        # tag id -> [(f, l), ...] while collecting
        self.raw = {}           # tag id -> [(tx, ty, tz, phi), ...] camera-frame, for the log
        self.t0 = time.monotonic()
        self.total_distance = 0.0
        self.total_rotation = 0.0
        self.slam = EkfSlam()

    # ---- camera ----------------------------------------------------------------------
    def on_camera_info(self, msg):
        self.camera_frames += 1

    def on_detections(self, msg):
        self.apriltag_frames += 1

    def on_tf(self, msg):
        if not self.collecting:
            return
        for tf in msg.transforms:
            if tf.header.frame_id != CAMERA_FRAME or not tf.child_frame_id.startswith('tag_'):
                continue
            try:
                tag_id = int(tf.child_frame_id[4:])
            except ValueError:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            f, l, phi = tag_in_robot_frame((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            self.frames.setdefault(tag_id, []).append((f, l))
            self.raw.setdefault(tag_id, []).append((t.x, t.y, t.z, phi))

    def frame_rates(self, seconds):
        cam, tag = self.camera_frames, self.apriltag_frames
        self.wait(seconds)
        return (self.camera_frames - cam) / seconds, (self.apriltag_frames - tag) / seconds

    # ---- log -------------------------------------------------------------------------
    def write(self, record):
        record['t'] = round(time.monotonic() - self.t0, 3)
        self.log_file.write(json.dumps(record) + '\n')
        self.log_file.flush()

    def state_record(self):
        return {'s': [round(v, 5) for v in self.slam.s.tolist()],
                'P': [[round(v, 8) for v in row] for row in self.slam.P.tolist()],
                'ids': list(self.slam.ids)}

    # ---- look / move -----------------------------------------------------------------
    def look(self, seconds=None):
        self.frames, self.raw = {}, {}
        cam, tag = self.camera_frames, self.apriltag_frames
        self.collecting = True
        self.wait(self.observe_time if seconds is None else seconds)
        self.collecting = False
        results = fuse_look(self.slam, self.frames)
        for tag_id, n, f, l, status in results:
            where = '' if f is None else f' at f={f:.2f} l={l:+.2f} m'
            self.get_logger().info(f'   tag {tag_id} ({n} frames){where}: {status}')
        x, y, th = self.slam.pose()
        Pr = self.slam.pose_cov()
        self.get_logger().info(
            f'   pose ({x:+.3f}, {y:+.3f}, {math.degrees(th):+6.1f} deg) '
            f'std {math.sqrt(Pr[0, 0]) * 100:.1f}/{math.sqrt(Pr[1, 1]) * 100:.1f} cm '
            f'{math.degrees(math.sqrt(Pr[2, 2])):.1f} deg, {len(self.slam.ids)} landmarks'
            + ('' if results else f'  (no tag; camera {self.camera_frames - cam}, '
                                  f'apriltag {self.apriltag_frames - tag} frames)'))
        self.write({'type': 'look',
                    'frames': {str(k): v for k, v in self.frames.items()},
                    'raw': {str(k): v for k, v in self.raw.items()},
                    'status': {str(r[0]): r[4] for r in results},
                    'pose': [x, y, th], 'pose_cov': Pr.tolist()})

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def move(self, kind, value):
        command, duration = turn_command(value) if kind == 'turn' else drive_command(value)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if not self.dry_run:
                self.send(*command)
            self.wait(min(PUBLISH_PERIOD_S, max(0.0, end - time.monotonic())))
        self.send(0.0, 0.0)
        if kind == 'turn':
            self.slam.predict_turn(value)
            self.total_rotation += abs(value)
        else:
            self.slam.predict_drive(value)
            self.total_distance += abs(value)
        self.write({'type': 'motion', 'kind': kind, 'value': value})
        self.wait(self.settle_time)

    def go_to(self, goal):
        for _ in range(MAX_STEPS):
            motion = next_motion(self.slam.pose(), goal, self.step)
            if motion is None:
                return
            kind, value = motion
            x, y, _ = self.slam.pose()
            self.get_logger().info(
                f' {kind} ' + (f'{math.degrees(value):+.1f} deg' if kind == 'turn'
                               else f'{value * 100:+.1f} cm')
                + f'  (point {math.hypot(goal[0] - x, goal[1] - y) * 100:.0f} cm away)')
            self.move(kind, value)
            self.look()
        self.get_logger().warn(f'Gave up on {goal} after {MAX_STEPS} steps')

    # ---- main ------------------------------------------------------------------------
    def run(self):
        path = load_path(self.path_file)
        if math.hypot(*path[0]) < 1e-6:
            path = path[1:]     # the start (0, 0) is where the robot already is
        os.makedirs(LOG_DIR, exist_ok=True)
        name = 'noise' if self.stationary_time > 0 else 'slam'
        log_path = os.path.join(LOG_DIR, time.strftime(f'{name}_%Y%m%d_%H%M%S.jsonl'))
        self.log_file = open(log_path, 'w')
        self.get_logger().info('slam_follower v1: EKF-SLAM, no map given, map frame = start pose')
        self.get_logger().info(f'Path ({self.path_file}), {self.loops} loops: {path}')
        self.write({'type': 'header', 'path_file': self.path_file, 'path': path,
                    'loops': self.loops, 'step': self.step, 'camera_offset': CAMERA_OFFSET,
                    'observe_time': self.observe_time, 'settle_time': self.settle_time,
                    'stationary_time': self.stationary_time,
                    'noise': {k: getattr(ekf_slam, k) for k in (
                        'DRIVE_STD_BASE', 'DRIVE_STD_PER_M', 'DRIVE_HEADING_STD_PER_M',
                        'TURN_STD_BASE', 'TURN_STD_FRACTION', 'MEAS_STD_BASE',
                        'MEAS_STD_PER_M', 'GATE_CHI2', 'MIN_FRAMES', 'MAX_RANGE')}})

        if not self.dry_run and self.stationary_time <= 0:
            self.get_logger().info('Waiting for motor_control to subscribe to motor_commands...')
            while self.publisher.get_subscription_count() == 0:
                self.wait(0.1)
            for i in range(3, 0, -1):
                self.get_logger().info(f'Starting in {i}...')
                self.wait(1.0)
        cam_fps, tag_fps = self.frame_rates(2.0)
        self.get_logger().info(f'Camera {cam_fps:.1f} fps, apriltag {tag_fps:.1f} fps')
        if tag_fps < 5.0:
            self.get_logger().warn(f'AprilTag gets only {tag_fps:.1f} frames/s (want ~10)')

        self.t0 = time.monotonic()
        if self.stationary_time > 0:
            # noise test: stand still, one long look per second, every frame logged
            self.get_logger().info(f'Stationary noise test, {self.stationary_time:.0f} s')
            end = time.monotonic() + self.stationary_time
            while time.monotonic() < end:
                self.look(1.0)
        else:
            self.get_logger().info('Start')
            self.look()
            for loop in range(1, self.loops + 1):
                for i, goal in enumerate(path, start=1):
                    self.get_logger().info(f'=== loop {loop}, point {i}/{len(path)}: {goal}')
                    self.go_to(goal)
                rec = self.state_record()
                rec.update({'type': 'loop', 'loop': loop})
                self.write(rec)
                self.get_logger().info(f'=== loop {loop} done. Map:')
                self.print_map()
        rec = self.state_record()
        rec['type'] = 'end'
        self.write(rec)
        self.log_file.close()
        self.get_logger().info(
            f'Done. Driven {self.total_distance:.2f} m, turned '
            f'{math.degrees(self.total_rotation):.0f} deg, {time.monotonic() - self.t0:.0f} s')
        self.print_map()
        self.get_logger().info(f'Log: {log_path}')

    def print_map(self):
        for tag_id in self.slam.ids:
            L, C = self.slam.landmark(tag_id)
            self.get_logger().info(
                f'   tag {tag_id}: ({L[0]:+.3f}, {L[1]:+.3f}) std '
                f'{math.sqrt(C[0, 0]) * 100:.1f}/{math.sqrt(C[1, 1]) * 100:.1f} cm, '
                f'{self.slam.updates[tag_id]} sightings')


def main(args=None):
    rclpy.init(args=args)
    node = SlamFollowerNode()
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
