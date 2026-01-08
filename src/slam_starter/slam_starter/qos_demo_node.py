#!/usr/bin/env python3
"""
ROS2 QoS Demo Node
Demonstrates different Quality of Service settings
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class QoSDemoNode(Node):
    """Node demonstrating QoS settings"""

    def __init__(self):
        super().__init__('qos_demo_node')
        
        # Define different QoS profiles
        
        # Best effort QoS (for sensors, lossy OK)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Reliable QoS (for critical data)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers with different QoS
        self.publisher_best_effort = self.create_publisher(
            String, 'topic_best_effort', qos_best_effort
        )
        self.publisher_reliable = self.create_publisher(
            String, 'topic_reliable', qos_reliable
        )
        
        # Create subscribers with matching QoS
        self.subscriber_best_effort = self.create_subscription(
            String, 'topic_best_effort', self.callback_best_effort, qos_best_effort
        )
        self.subscriber_reliable = self.create_subscription(
            String, 'topic_reliable', self.callback_reliable, qos_reliable
        )
        
        # Timer to publish messages
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('QoS Demo Node started')
        self.get_logger().info('Publishing to topic_best_effort (BEST_EFFORT)')
        self.get_logger().info('Publishing to topic_reliable (RELIABLE)')

    def timer_callback(self):
        """Publish messages with different QoS"""
        msg = String()
        
        # Publish with best effort
        msg.data = f'Best Effort message #{self.counter}'
        self.publisher_best_effort.publish(msg)
        
        # Publish with reliable
        msg.data = f'Reliable message #{self.counter}'
        self.publisher_reliable.publish(msg)
        
        self.counter += 1

    def callback_best_effort(self, msg):
        """Callback for best effort topic"""
        self.get_logger().info(f'Received (best effort): {msg.data}')

    def callback_reliable(self, msg):
        """Callback for reliable topic"""
        self.get_logger().info(f'Received (reliable): {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = QoSDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
