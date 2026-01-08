#!/usr/bin/env python3
"""
ROS2 Composable Node
A node that can be composed into a container
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ComposablePublisher(Node):
    """Composable publisher node"""

    def __init__(self):
        super().__init__('composable_publisher')
        self.publisher_ = self.create_publisher(String, 'composed_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Composable Publisher initialized')

    def timer_callback(self):
        """Publish messages"""
        msg = String()
        msg.data = f'Composed message #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1


class ComposableSubscriber(Node):
    """Composable subscriber node"""

    def __init__(self):
        super().__init__('composable_subscriber')
        self.subscription = self.create_subscription(
            String,
            'composed_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('Composable Subscriber initialized')

    def listener_callback(self, msg):
        """Receive messages"""
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    """Main function to run both nodes"""
    rclpy.init(args=args)
    
    # Create both nodes
    publisher_node = ComposablePublisher()
    subscriber_node = ComposableSubscriber()
    
    # Create executor and add both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
