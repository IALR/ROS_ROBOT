import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.twist = Twist()
        self.twist.linear.x = 0.2  # forward speed
        self.twist.angular.z = 0.0

    def timer_callback(self):
        self.pub.publish(self.twist)

def main():
    rclpy.init()
    node = MoveForward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
