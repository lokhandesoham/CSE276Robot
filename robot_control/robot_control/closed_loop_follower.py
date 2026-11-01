"""Closed-loop waypoint follower: stop-and-go with AprilTag position fixes.

Usage (camera with image_rectify:=true, apriltag_node and motor_control running):
    ros2 run robot_control closed_loop_follower
    ros2 run robot_control closed_loop_follower --ros-args -p step:=0.25 -p settle_time:=1.0
    ros2 run robot_control closed_loop_follower --ros-args -p dry_run:=true   # no motors

The robot knows the map (tag poses in landmarks.txt) and the waypoints; at every stop it
works out where it is from the tags, then plans the next motion from there:
      1. look       stand still, collect AprilTag poses for observe_time seconds and fuse
                    them with the predicted pose (localization.py)
      2. search     the robot only drives if the stop right before the drive saw a tag. If
                    not: turn toward the tags the map says are visible, then sweep a full
                    circle in SEARCH_STEP turns, looking after each, until a tag is seen;
                    then re-plan from there. The one drive after turning back toward the
                    waypoint from a successful search is allowed without a new sighting
                    (the tag found may be out of view again). A full circle with no tag
                    stops the run.
      3. decide     far from the waypoint: turn toward it if pointing more than AIM_TOL off,
                    else drive toward it, at most `step` meters
                    close to it (APPROACH_RANGE): no more turning (a few cm sideways is a large
                    angle there), only drive the forward/backward part of the remaining offset
      4. move       play the motion with the HW1 calibrated model (calibration.py) and
                    predict the new pose with the unicycle model. Turns over 90 deg are
                    split into equal pieces of at most 90 deg (the calibration was checked
                    at 90 deg), with the settle pause after each piece
    Then turn to the waypoint heading, look again, and re-approach once if the turn left the
    robot off the mark. The final heading turn itself needs no fix (at (1, 2) facing pi no tag
    can be in view).
    Motions shorter than MIN_DRIVE / MIN_TURN are not sent: the motors do not move reliably
    for such short commands, and the loop would repeat them forever.

Output:
    ~/hw2_runs/run_<time>.csv  one row per 50 ms while moving ('move'), per fix ('fix') and
                               per waypoint reached ('waypoint'); plot with
                               tools/plot_trajectory.py
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control.calibration import drive_command, turn_command
from robot_control.localization import (
    PoseEstimate, load_landmarks, median_sighting, tag_in_robot_frame, tag_view_turns, wrap_angle)
from robot_control.waypoint_follower import load_waypoints


PUBLISH_PERIOD_S = 0.05
CAMERA_FRAME = 'camera_frame'

POS_TOL = 0.05                  # m, waypoint reached
APPROACH_RANGE = 0.15           # m, closer than this only drive forward/backward, no turns
AIM_TOL = math.radians(5.0)     # turn toward the waypoint when pointing further off than this
FACE_TOL = math.radians(5.0)    # final heading tolerance
MIN_DRIVE = 0.04                # m, 0.15 s: shorter drives barely move the robot
MIN_TURN = math.radians(5.0)    # 0.13 s: shorter turns are mostly the 0.07 s start-up delay
MAX_STEPS = 30                  # per waypoint, in case it never converges
MAX_FACE_ROUNDS = 2             # turn-to-heading / re-approach rounds per waypoint
MIN_FRAMES = 2                  # a tag must be seen in this many frames to be used
MAX_TURN = math.radians(95.0)   # longer turns are split into pieces of at most 90 deg

SEARCH_MAX_TURNS = 2            # map-predicted tags to try before sweeping
SEARCH_STEP = math.radians(40.0)    # sweep step, under the 64 deg field of view so no gaps

DEFAULT_ROOT = os.path.expanduser('~/ros2_ws/src/rubikpi_ros2')
DEFAULT_WAYPOINTS = os.path.join(DEFAULT_ROOT, 'waypoints_hw2.txt')
DEFAULT_LANDMARKS = os.path.join(DEFAULT_ROOT, 'landmarks.txt')
LOG_DIR = os.path.expanduser('~/hw2_runs')


def next_motion(pose, goal, step):
    """('turn', angle), ('drive', distance) or None when (x, y) of goal is reached"""
    x, y, theta = pose
    dx, dy = goal[0] - x, goal[1] - y
    dist = math.hypot(dx, dy)
    if dist < POS_TOL:
        return None
    err = wrap_angle(math.atan2(dy, dx) - theta)

    if dist < APPROACH_RANGE:
        # forward/backward part of the offset only; the sideways part is accepted
        along = dist * math.cos(err)
        return ('drive', along) if abs(along) >= MIN_DRIVE else None

    if abs(err) > AIM_TOL:
        return ('turn', err)
    # the last step goes all the way instead of leaving a short remainder
    along = dist * math.cos(err)
    return ('drive', along if along < step + APPROACH_RANGE else step)


class NoTagError(Exception):
    pass


class ClosedLoopFollowerNode(Node):
    def __init__(self):
        super().__init__('closed_loop_follower_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.waypoints_file = self.declare_parameter('waypoints_file', DEFAULT_WAYPOINTS).value
        self.landmarks_file = self.declare_parameter('landmarks_file', DEFAULT_LANDMARKS).value
        self.step = self.declare_parameter('step', 0.25).value
        self.settle_time = self.declare_parameter('settle_time', 1.0).value
        self.observe_time = self.declare_parameter('observe_time', 0.8).value
        self.dry_run = self.declare_parameter('dry_run', False).value

        self.collecting = False
        self.sightings = {}     # tag id -> [(forward, left, phi), ...] while collecting
        self.seen_at_last_stop = False  # the latest localize() used a tag
        self.search_fix = False         # a search found a tag and no drive has happened since
        self.t0 = time.monotonic()
        self.total_distance = 0.0
        self.total_rotation = 0.0

    # ---- camera ----------------------------------------------------------------------
    def on_tf(self, msg):
        if not self.collecting:
            return
        for tf in msg.transforms:
            if tf.header.frame_id != CAMERA_FRAME or not tf.child_frame_id.startswith('tag_'):
                continue
            tag_id = int(tf.child_frame_id[4:])
            if tag_id not in self.landmarks:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            self.sightings.setdefault(tag_id, []).append(
                tag_in_robot_frame((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))

    def localize(self):
        """Collect tag sightings while standing still and fuse them. Returns the tag ids used."""
        self.sightings = {}
        self.collecting = True
        self.wait(self.observe_time)
        self.collecting = False

        used = []
        for tag_id, sightings in sorted(self.sightings.items()):
            if len(sightings) < MIN_FRAMES:
                continue
            status = self.est.correct(self.landmarks[tag_id], median_sighting(sightings))
            self.get_logger().info(f'   tag {tag_id} ({len(sightings)} frames): {status}')
            if status.startswith('used'):
                used.append(tag_id)
        x, y, theta = self.est.pose()
        self.get_logger().info(
            f'   pose ({x:+.3f}, {y:+.3f}, {math.degrees(theta):+6.1f} deg) '
            f'std {math.sqrt(self.est.var_xy) * 100:.1f} cm / '
            f'{math.degrees(math.sqrt(self.est.var_theta)):.1f} deg'
            + ('' if used else '  (no tag, dead reckoning)'))
        self.log('fix', ' '.join(f'tag{i}' for i in used))
        self.seen_at_last_stop = bool(used)
        return used

    def search(self):
        """Turn until a tag is seen: first toward the tags the map says are visible, then a
        full circle in SEARCH_STEP turns. Raises NoTagError if nothing is found."""
        turns = [t for t in tag_view_turns(self.est.pose(), self.landmarks) if abs(t[0]) >= MIN_TURN]
        for angle, tag_id in turns[:SEARCH_MAX_TURNS]:
            self.get_logger().info(f' search: turn {math.degrees(angle):+.1f} deg toward tag {tag_id}')
            self.move('turn', angle)
            if self.localize():
                self.search_fix = True
                return
        turned = 0.0
        while turned < 2 * math.pi:
            self.get_logger().info(f' search: sweep, turn {math.degrees(SEARCH_STEP):+.0f} deg')
            self.move('turn', SEARCH_STEP)
            turned += SEARCH_STEP
            if self.localize():
                self.search_fix = True
                return
        raise NoTagError('no tag seen in a full circle')

    # ---- motors ----------------------------------------------------------------------
    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def wait(self, seconds):
        """Keep processing ROS callbacks for the given time"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def move(self, kind, value):
        """Play one turn/drive, logging the in-between pose, then update the prediction"""
        if kind == 'turn' and abs(value) > MAX_TURN:
            pieces = math.ceil(abs(value) / math.radians(90.0))
            for _ in range(pieces):
                self.move('turn', value / pieces)
            return
        command, duration = turn_command(value) if kind == 'turn' else drive_command(value)
        x0, y0, th0 = self.est.pose()
        start = time.monotonic()
        end = start + duration
        while time.monotonic() < end:
            f = (time.monotonic() - start) / duration
            if kind == 'turn':
                self.log('move', pose=(x0, y0, wrap_angle(th0 + f * value)))
            else:
                self.log('move', pose=(x0 + f * value * math.cos(th0),
                                       y0 + f * value * math.sin(th0), th0))
            if not self.dry_run:
                self.send(*command)
            self.wait(min(PUBLISH_PERIOD_S, end - time.monotonic()))
        self.send(0.0, 0.0)

        if kind == 'turn':
            self.est.predict_turn(value)
            self.total_rotation += abs(value)
        else:
            self.est.predict_drive(value)
            self.total_distance += abs(value)
            self.search_fix = False
        self.log('move')
        self.wait(self.settle_time)

    # ---- log -------------------------------------------------------------------------
    def log(self, event, info='', pose=None):
        x, y, theta = pose if pose is not None else self.est.pose()
        self.writer.writerow([f'{time.monotonic() - self.t0:.2f}', event,
                              f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}', info])

    # ---- main ------------------------------------------------------------------------
    def go_to(self, goal):
        for _ in range(MAX_STEPS):
            motion = next_motion(self.est.pose(), goal, self.step)
            if motion is None:
                return
            kind, value = motion
            if kind == 'drive' and not (self.seen_at_last_stop or self.search_fix):
                self.get_logger().info(' no tag at this stop: searching before driving')
                self.search()
                continue    # the search turned the robot: re-plan from the fix
            dist = math.hypot(goal[0] - self.est.x, goal[1] - self.est.y)
            self.get_logger().info(
                f' {kind} ' + (f'{math.degrees(value):+.1f} deg' if kind == 'turn'
                               else f'{value * 100:+.1f} cm')
                + f'  (waypoint {dist * 100:.0f} cm away)')
            self.move(kind, value)
            self.localize()
        self.get_logger().warn(f'Gave up on {goal} after {MAX_STEPS} steps')

    def face(self, goal):
        """Turn to the goal heading; re-approach if the turn left the robot off the mark"""
        for _ in range(MAX_FACE_ROUNDS):
            err = wrap_angle(goal[2] - self.est.theta)
            if abs(err) < max(FACE_TOL, MIN_TURN):
                return
            self.get_logger().info(f' face: turn {math.degrees(err):+.1f} deg to the waypoint heading')
            self.move('turn', err)
            self.localize()
            if math.hypot(goal[0] - self.est.x, goal[1] - self.est.y) > POS_TOL:
                self.go_to(goal)

    def run(self):
        waypoints = load_waypoints(self.waypoints_file)
        self.landmarks = load_landmarks(self.landmarks_file)
        self.est = PoseEstimate(*waypoints[0])
        self.get_logger().info('closed_loop_follower v4: drives only right after a tag sighting, '
                               'turns over 90 deg split')
        self.get_logger().info(f'Waypoints: {waypoints}')
        self.get_logger().info('Landmarks: ' + ', '.join(
            f'tag {i} at ({x}, {y}) facing {math.degrees(f):.0f} deg'
            for i, (x, y, f) in sorted(self.landmarks.items())))
        self.get_logger().info(f'Start pose {waypoints[0]}: turning centre on the mark')

        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, time.strftime('run_%Y%m%d_%H%M%S.csv'))
        log_file = open(log_path, 'w', newline='')
        self.writer = csv.writer(log_file)
        self.writer.writerow(['t', 'event', 'x', 'y', 'theta', 'info'])

        if not self.dry_run:
            self.get_logger().info('Waiting for motor_control to subscribe to motor_commands...')
            while self.publisher.get_subscription_count() == 0:
                self.wait(0.1)
            for i in range(3, 0, -1):
                self.get_logger().info(f'Starting in {i}...')
                self.wait(1.0)

        self.t0 = time.monotonic()
        self.get_logger().info('Start')
        self.localize()
        results = []
        try:
            for i, goal in enumerate(waypoints[1:], start=1):
                self.get_logger().info(f'=== waypoint {i}: {goal}')
                self.go_to(goal)
                self.face(goal)
                x, y, theta = self.est.pose()
                err_pos = math.hypot(goal[0] - x, goal[1] - y)
                err_theta = wrap_angle(goal[2] - theta)
                results.append((i, goal, (x, y, theta), err_pos, err_theta))
                self.log('waypoint', f'wp{i}')
                self.get_logger().info(
                    f'=== reached waypoint {i}: estimate ({x:+.3f}, {y:+.3f}, '
                    f'{math.degrees(theta):+.1f} deg), estimated error {err_pos * 100:.1f} cm, '
                    f'{math.degrees(err_theta):+.1f} deg')
        except NoTagError as e:
            self.send(0.0, 0.0)
            self.get_logger().error(f'Stopped: {e}. Check the camera and apriltag panes '
                                    '(is /detections publishing? are the tags at lens height?)')

        log_file.close()
        self.get_logger().info('Done. Estimated error at each waypoint (measure the true error with a tape):')
        for i, goal, _, err_pos, err_theta in results:
            self.get_logger().info(
                f'  waypoint {i} {goal}: {err_pos * 100:.1f} cm, {math.degrees(err_theta):+.1f} deg')
        self.get_logger().info(
            f'Driven {self.total_distance:.2f} m, turned {math.degrees(self.total_rotation):.0f} deg, '
            f'{time.monotonic() - self.t0:.0f} s')
        self.get_logger().info(f'Log: {log_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopFollowerNode()
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
