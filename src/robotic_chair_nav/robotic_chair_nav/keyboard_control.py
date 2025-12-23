#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

try:
    from pynput import keyboard
    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False
    print("WARNING: pynput not installed. Install with: pip install pynput")

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('front_half_angle_deg', 30.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.front_half_angle = math.radians(self.get_parameter('front_half_angle_deg').value)

        # Keyboard state
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.scan_received = False
        self.log_count = 0

        # Publisher / Subscriber
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        # Timer for command publishing
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Keyboard Control node started")
        self.get_logger().info(f"Stop distance: {self.stop_distance}m")
        self.get_logger().info(f"Front sector angle: ±{math.degrees(self.front_half_angle):.1f}°")
        self.print_help()

        # Start keyboard listener if pynput is available
        if HAS_PYNPUT:
            self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
            self.listener.start()

    def print_help(self):
        print("\n" + "="*50)
        print("ROBOTIC CHAIR KEYBOARD CONTROL")
        print("="*50)
        print("Controls:")
        print("  W/w     : Move Forward")
        print("  S/s     : Move Backward")
        print("  A/a     : Turn Left")
        print("  D/d     : Turn Right")
        print("  SPACE   : Stop")
        print("  Q/q     : Quit")
        print("="*50)
        print("Status: Ready for input (obstacle detection: ON)")
        print("="*50 + "\n")

    def on_key_press(self, key):
        """Handle key press events"""
        try:
            char = key.char.lower() if hasattr(key, 'char') else None
            
            if char == 'w':
                self.forward = True
                self.backward = False
            elif char == 's':
                self.backward = True
                self.forward = False
            elif char == 'a':
                self.left = True
                self.right = False
            elif char == 'd':
                self.right = True
                self.left = False
            elif char == 'q':
                self.get_logger().info("Shutting down...")
                rclpy.shutdown()
            elif key == keyboard.Key.space:
                self.forward = False
                self.backward = False
                self.left = False
                self.right = False
        except AttributeError:
            pass

    def on_key_release(self, key):
        """Handle key release events"""
        try:
            char = key.char.lower() if hasattr(key, 'char') else None
            
            if char == 'w':
                self.forward = False
            elif char == 's':
                self.backward = False
            elif char == 'a':
                self.left = False
            elif char == 'd':
                self.right = False
        except AttributeError:
            pass

    def scan_callback(self, msg: LaserScan):
        """Check for obstacles in front sector"""
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info("LaserScan data received!")
            self.get_logger().info(f"  - Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°")
            self.get_logger().info(f"  - Distance range: {msg.range_min:.2f}m to {msg.range_max:.2f}m")
            self.get_logger().info(f"  - Number of ranges: {len(msg.ranges)}")

        if not msg.ranges or msg.angle_increment == 0.0:
            self.obstacle_detected = False
            self.min_distance = float('inf')
            return

        # Compute indices for front sector (0 degrees is straight ahead)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        start_angle = -self.front_half_angle
        end_angle = self.front_half_angle
        
        start_idx = max(0, int((start_angle - angle_min) / angle_inc))
        end_idx = min(len(msg.ranges) - 1, int((end_angle - angle_min) / angle_inc))

        if start_idx > end_idx:
            self.obstacle_detected = False
            self.min_distance = float('inf')
            return

        sector = msg.ranges[start_idx:end_idx + 1]

        # Filter invalid values
        valid = [
            r for r in sector
            if not math.isinf(r)
            and not math.isnan(r)
            and msg.range_min < r < msg.range_max
        ]

        if valid:
            self.min_distance = min(valid)
            self.obstacle_detected = self.min_distance < self.stop_distance
            
            # Log every 10 times to avoid spam
            self.log_count += 1
            if self.log_count >= 10:
                self.log_count = 0
                self.get_logger().info(f"Min distance: {self.min_distance:.2f}m | Obstacle: {self.obstacle_detected}")
        else:
            self.obstacle_detected = False
            self.min_distance = float('inf')

    def timer_callback(self):
        """Publish velocity commands"""
        twist = Twist()

        # Check obstacle before moving forward
        if self.forward and self.obstacle_detected:
            self.get_logger().warn(f"⚠️  OBSTACLE DETECTED at {self.min_distance:.2f}m - STOPPING!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # Linear velocity
            if self.forward:
                twist.linear.x = self.linear_speed
            elif self.backward:
                twist.linear.x = -self.linear_speed
            else:
                twist.linear.x = 0.0

            # Angular velocity
            if self.left:
                twist.angular.z = self.angular_speed
            elif self.right:
                twist.angular.z = -self.angular_speed
            else:
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main():
    if not HAS_PYNPUT:
        print("\n" + "="*60)
        print("ERROR: pynput library not found!")
        print("="*60)
        print("Install it with:")
        print("  pip install pynput")
        print("="*60 + "\n")
        return

    rclpy.init()
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()