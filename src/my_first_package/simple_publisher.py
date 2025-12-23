import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    A simple publisher node that sends messages every second.
    """
    
    def __init__(self):
        # Initialize the node with name 'simple_publisher'
        super().__init__('simple_publisher')
        
        # Create a publisher
        # Parameters: message_type, topic_name, queue_size
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        
        # Create a timer that calls timer_callback every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Message counter
        self.counter = 0
        
        # Log startup message
        self.get_logger().info('Publisher node started!')

    def timer_callback(self):
        """
        This function is called every second by the timer.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello ROS2! Message #{self.counter}'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log to console
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment counter
        self.counter += 1


def main(args=None):
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create our node
    node = SimplePublisher()
    
    # Keep the node running (and calling timer callbacks)
    rclpy.spin(node)
    
    # Cleanup when done
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()