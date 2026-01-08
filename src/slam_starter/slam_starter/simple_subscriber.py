#!/usr/bin/env python3
"""
Simple ROS2 Subscriber Node
Subscribes to string messages from a topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """A simple subscriber that receives string messages"""

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Simple Subscriber Node started')

    def listener_callback(self, msg):
        """Callback function when a message is received"""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
