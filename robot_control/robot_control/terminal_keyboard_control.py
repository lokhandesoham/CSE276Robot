import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# key -> (L, R) wheel commands
KEY_BINDINGS = {
    'w': (0.3, 0.3),
    's': (-0.3, -0.3),
    'a': (-0.5, 0.5),
    'd': (0.5, -0.5),
    'x': (0.0, 0.0),
    ' ': (0.0, 0.0),
}


class TerminalKeyboardControllerNode(Node):
    """Keyboard teleop that reads keys from the terminal (works over SSH, no X server needed).

    Terminals do not report key releases, so commands are latched:
    tap a key to change motion, tap X or SPACE to stop.
    """

    def __init__(self):
        super().__init__('keyboard_controller_node')

        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        self.current_L = 0.0
        self.current_R = 0.0
        self.running = True

        # put the terminal in cbreak mode so keys arrive immediately, without Enter
        self.old_term_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.key_thread = threading.Thread(target=self.read_keys, daemon=True)
        self.key_thread.start()

        # publish commands at 20Hz
        self.timer = self.create_timer(0.05, self.publish_motor_commands)

        self.print_instructions()

    def print_instructions(self):
        print("""
========================================
   Robot Keyboard Controller (terminal)
----------------------------------------
   W : Forward     (L=0.3,  R=0.3)
   S : Backward    (L=-0.3, R=-0.3)
   A : Turn Left   (L=-0.5, R=0.5)
   D : Turn Right  (L=0.5,  R=-0.5)
   X / SPACE : Stop
   Q / Ctrl-C : Quit
  Commands are latched: tap a key once, the robot keeps doing it until you stop it.
========================================
""", flush=True)

    def read_keys(self):
        """Read single key presses from stdin in a background thread"""
        while self.running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            c = sys.stdin.read(1).lower()

            if c == 'q' or c == '\x03':  # q or Ctrl-C
                self.running = False
                break
            if c in KEY_BINDINGS:
                self.current_L, self.current_R = KEY_BINDINGS[c]
                self.get_logger().info(f'Motor command: L={self.current_L:.2f}, R={self.current_R:.2f}')

    def publish_motor_commands(self):
        msg = Float32MultiArray()
        msg.data = [self.current_L, self.current_R]
        self.publisher.publish(msg)

    def stop_robot(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        self.publisher.publish(msg)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term_settings)


def main(args=None):
    if not sys.stdin.isatty():
        print('terminal_keyboard_control must be run in an interactive terminal: '
              'ros2 run robot_control terminal_keyboard_control', file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = TerminalKeyboardControllerNode()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.restore_terminal()
        node.stop_robot()
        node.get_logger().info('Stop command sent')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
