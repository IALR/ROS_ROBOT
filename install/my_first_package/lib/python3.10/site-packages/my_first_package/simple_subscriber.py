#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    A simple subscriber node that listens to messages.
    """
    
    def __init__(self):
        # Initialize the node with name 'simple_subscriber'
        super().__init__('simple_subscriber')
        
        # Create a subscriber
        # Parameters: message_type, topic_name, callback_function, queue_size
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10
        )
        
        # Log startup message
        self.get_logger().info('Subscriber node started! Waiting for messages...')

    def listener_callback(self, msg):
        """
        This function is called automatically whenever a message arrives.
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create our node
    node = SimpleSubscriber()
    
    # Keep the node running (and receiving messages)
    rclpy.spin(node)
    
    # Cleanup when done
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()