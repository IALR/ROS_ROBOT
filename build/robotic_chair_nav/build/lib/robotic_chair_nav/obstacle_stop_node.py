import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStop(Node):
    def __init__(self):
        super().__init__('obstacle_stop')

        # Parameters
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('front_half_angle_deg', 30.0)  # half-angle of the front sector

        self.forward_speed = self.get_parameter('forward_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.front_half_angle = math.radians(self.get_parameter('front_half_angle_deg').value)

        # Publisher / Subscriber
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info("ObstacleStop node started")

    def scan_callback(self, msg: LaserScan):
        # If scan is empty, stop
        if not msg.ranges:
            self._publish_stop()
            return

        # Compute indices for ±front_half_angle around 0 rad (front)
        # angle at index i: angle_min + i * angle_increment
        # front is assumed at 0 rad in ROS LaserScan convention
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        if angle_inc == 0.0:
            self._publish_stop()
            return

        start_angle = -self.front_half_angle
        end_angle = self.front_half_angle
        start_idx = max(0, int((start_angle - angle_min) / angle_inc))
        end_idx = min(len(msg.ranges) - 1, int((end_angle - angle_min) / angle_inc))

        if start_idx > end_idx:
            self._publish_stop()
            return

        sector = msg.ranges[start_idx:end_idx + 1]

        # Filter invalid values
        valid = [
            r for r in sector
            if not math.isinf(r)
            and not math.isnan(r)
            and msg.range_min < r < msg.range_max
        ]

        if valid and min(valid) < self.stop_distance:
            self._publish_stop()
        else:
            self._publish_forward()

    def _publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def _publish_forward(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = ObstacleStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()