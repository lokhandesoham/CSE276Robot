"""Save camera frames for checkerboard calibration: press Enter to save the latest frame.

Usage (camera node running with image_rectify:=false, i.e. raw distorted images):
    ros2 run robot_control capture_frames                 # saves to ~/calib_imgs
    ros2 run robot_control capture_frames OUT_DIR

Type q then Enter to quit. Frames are written as lossless .ppm files (no OpenCV needed
on the robot); copy the folder to the Mac and run tools/calibrate_camera.py on it.
"""
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


DEFAULT_DIR = os.path.expanduser('~/calib_imgs')
TOPIC = '/camera/image_raw'


class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_frames')
        self.lock = threading.Lock()
        self.latest = None
        self.create_subscription(Image, TOPIC, self.on_image, 1)

    def on_image(self, msg):
        with self.lock:
            self.latest = msg

    def take(self):
        with self.lock:
            return self.latest


def to_array(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    # PPM stores RGB; the colour order doesn't matter for calibration, but keep it right anyway
    return img if msg.encoding == 'rgb8' else img[:, :, ::-1]


def write_ppm(path, img):
    h, w, _ = img.shape
    with open(path, 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (w, h))
        f.write(np.ascontiguousarray(img).tobytes())


def main(args=None):
    argv = remove_ros_args(sys.argv)[1:]
    out_dir = os.path.expanduser(argv[0]) if argv else DEFAULT_DIR
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init(args=args)
    node = CaptureNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    count = len([f for f in os.listdir(out_dir) if f.endswith('.ppm')])
    print(f'Saving to {out_dir} ({count} frames already there). Enter = save, q + Enter = quit.')
    try:
        while True:
            if input().strip().lower() == 'q':
                break
            msg = node.take()
            if msg is None:
                print(f'no image received yet on {TOPIC}, is the camera node running?')
                continue
            path = os.path.join(out_dir, f'frame_{count:03d}.ppm')
            write_ppm(path, to_array(msg))
            count += 1
            print(f'saved {path} ({msg.width}x{msg.height}), {count} total')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
