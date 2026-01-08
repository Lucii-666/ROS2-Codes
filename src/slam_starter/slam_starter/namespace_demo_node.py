#!/usr/bin/env python3
"""
ROS2 Namespace Demo Node
Demonstrates namespace handling in ROS2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NamespaceDemo(Node):
    """Node demonstrating namespace usage"""

    def __init__(self):
        super().__init__('namespace_demo')
        
        # Get node's namespace
        namespace = self.get_namespace()
        node_name = self.get_name()
        fully_qualified_name = self.get_fully_qualified_name()
        
        self.get_logger().info(f'Node namespace: {namespace}')
        self.get_logger().info(f'Node name: {node_name}')
        self.get_logger().info(f'Fully qualified name: {fully_qualified_name}')
        
        # Create publisher in current namespace
        self.publisher = self.create_publisher(String, 'namespaced_topic', 10)
        
        # Create subscriber in current namespace
        self.subscription = self.create_subscription(
            String,
            'namespaced_topic',
            self.callback,
            10
        )
        
        # Timer to publish
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Namespace Demo Node started')

    def timer_callback(self):
        """Publish namespaced messages"""
        msg = String()
        msg.data = f'Namespaced message from {self.get_namespace()}: #{self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

    def callback(self, msg):
        """Receive namespaced messages"""
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = NamespaceDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
