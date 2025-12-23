#!/usr/bin/env python3
import math
import sys
import threading
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

KEY_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    ' ': (0.0, 0.0),
}

class SafetyTeleop(Node):
    def __init__(self):
        super().__init__('safety_teleop')
        # Params
        self.declare_parameter('lin_speed', 0.4)
        self.declare_parameter('ang_speed', 1.2)
        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.lin_speed = float(self.get_parameter('lin_speed').value)
        self.ang_speed = float(self.get_parameter('ang_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value

        self.desired_twist = Twist()
        self.safe_to_move = True

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_timer(0.05, self.pub_cb)  # 20 Hz

        self._start_keyboard_thread()
        self.get_logger().info("SafetyTeleop running. Use WASD, space to stop, Ctrl+C to quit.")

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges or msg.angle_increment == 0.0:
            self.safe_to_move = False
            return
        start = max(0, int(( -self.front_half_angle - msg.angle_min) / msg.angle_increment))
        end = min(len(msg.ranges) - 1, int(( self.front_half_angle - msg.angle_min) / msg.angle_increment))
        sector = msg.ranges[start:end + 1]
        valid = [
            r for r in sector
            if not math.isinf(r) and not math.isnan(r)
            and msg.range_min < r < msg.range_max
        ]
        self.safe_to_move = not (valid and min(valid) < self.stop_distance)

    def pub_cb(self):
        twist = Twist()
        if self.safe_to_move:
            twist = self.desired_twist
        self.cmd_pub.publish(twist)

    def _start_keyboard_thread(self):
        thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        thread.start()

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[ch]
                    self.desired_twist.linear.x = lin * self.lin_speed
                    self.desired_twist.angular.z = ang * self.ang_speed
                elif ch in ('\x03',):  # Ctrl+C
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    node = SafetyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()