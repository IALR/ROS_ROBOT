import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStop(Node):
    def __init__(self):
        super().__init__('obstacle_stop')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('stop_distance', 0.6)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

    def scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        min_r = min(valid) if valid else float('inf')
        twist = Twist()
        if min_r < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
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