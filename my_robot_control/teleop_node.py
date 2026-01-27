from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

import threading
import sys
import termios
import tty
import time


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.get_logger().info("Teleop node started (WASD hold-to-move)")

        # --- Desired motion "state" (the timer publishes these) ---
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # --- Safety limits (clamp) ---
        self.max_linear_speed = 0.2
        self.max_angular_speed = 2.5
        self.min_linear_speed = -self.max_linear_speed
        self.min_angular_speed = -self.max_angular_speed

        # --- Teleop "feel" parameters ---
        self.normal_forward = 0.20
        self.reverse_speed = -0.20
        self.turn_speed = 0.50

        # Deadman timeout: if we haven't *seen* a key recently, we stop that axis
        # (We simulate "key release" using time since last key repeat event.)
        self.key_timeout = 0.15  # seconds (0.10–0.20 is a good range)

        # Track last time we saw forward/back keys vs turning keys (independent)
        # This enables: hold W to drive while letting go of A/D stops turning.
        now = time.time()
        self.last_w_time = now
        self.last_turn_time = now

        # Publisher to robot command velocity
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer publishes at a steady rate (here: 20 Hz)
        # Publishing steadily is smoother than publishing only on keypress.
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Keyboard input must NOT block rclpy.spin(), so run it in a separate thread.
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

    def timer_callback(self):
        """Runs at a fixed rate. Applies deadman timeout + clamps + publishes /cmd_vel."""
        now = time.time()

        # --- Deadman behavior (simulate key release) ---
        
        # If forward/back keys haven't been seen recently -> stop linear motion
        if (now - self.last_w_time) > self.key_timeout:
            self.linear_speed = 0.0

        # If turn keys haven't been seen recently -> stop turning
        if (now - self.last_turn_time) > self.key_timeout:
            self.angular_speed = 0.0

        # --- Clamp to safe limits ---
        lin = self.linear_speed
        if lin > self.max_linear_speed:
            lin = self.max_linear_speed
        elif lin < self.min_linear_speed:
            lin = self.min_linear_speed

        ang = self.angular_speed
        if ang > self.max_angular_speed:
            ang = self.max_angular_speed
        elif ang < self.min_angular_speed:
            ang = self.min_angular_speed

        # --- Publish Twist (cmd_vel) ---
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_vel_pub.publish(msg)

    def keyboard_loop(self):
        """
        Reads one character at a time in raw terminal mode.
        Updates state + timestamps so holding keys keeps motion active.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            # Raw mode lets us read single key presses without Enter
            tty.setraw(fd)

            while rclpy.ok(): 
                key = sys.stdin.read(1)
                now = time.time()

                # --- Forward / reverse (W/S) ---
                if key == "w":
                    self.linear_speed = self.normal_forward
                    self.last_w_time = now

                elif key == "s":
                    self.linear_speed = self.reverse_speed
                    self.last_w_time = now

                # --- Turning (hold A/D to turn) ---
                elif key == "a":
                    self.angular_speed = self.turn_speed
                    self.last_turn_time = now
                elif key == "d":
                    self.angular_speed = -self.turn_speed
                    self.last_turn_time = now

                # --- Emergency stop ---
                elif key == " ":
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0

                    # Also "age out" the timestamps so deadman keeps it stopped
                    self.last_w_time = 0.0
                    self.last_turn_time = 0.0

                # --- Quit ---
                elif key == "q":
                    # Stop robot before quitting input loop
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    break

        finally:
            # ALWAYS restore terminal settings, or your terminal will act weird after exit
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: publish a final zero command before shutting down
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


