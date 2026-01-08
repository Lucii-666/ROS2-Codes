#!/usr/bin/env python3
"""
ROS2 Topic Statistics Node
Monitors and reports topic statistics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopicStatisticsNode(Node):
    """Node that monitors topic statistics"""

    def __init__(self):
        super().__init__('topic_statistics_node')
        
        # Subscribe to topic
        self.subscription = self.create_subscription(
            String,
            'monitored_topic',
            self.callback,
            10
        )
        
        # Statistics
        self.message_count = 0
        self.last_message_time = None
        self.min_interval = float('inf')
        self.max_interval = 0.0
        self.total_interval = 0.0
        
        # Timer for reporting
        self.report_timer = self.create_timer(5.0, self.report_statistics)
        
        self.get_logger().info('Topic Statistics Node started')
        self.get_logger().info('Monitoring: /monitored_topic')

    def callback(self, msg):
        """Process incoming messages and update statistics"""
        current_time = self.get_clock().now()
        self.message_count += 1
        
        if self.last_message_time is not None:
            interval = (current_time - self.last_message_time).nanoseconds / 1e9
            self.total_interval += interval
            self.min_interval = min(self.min_interval, interval)
            self.max_interval = max(self.max_interval, interval)
        
        self.last_message_time = current_time

    def report_statistics(self):
        """Report accumulated statistics"""
        if self.message_count == 0:
            self.get_logger().info('No messages received yet')
            return
        
        self.get_logger().info('=== Topic Statistics ===')
        self.get_logger().info(f'Total messages: {self.message_count}')
        
        if self.message_count > 1:
            avg_interval = self.total_interval / (self.message_count - 1)
            avg_rate = 1.0 / avg_interval if avg_interval > 0 else 0.0
            
            self.get_logger().info(f'Average interval: {avg_interval:.3f}s')
            self.get_logger().info(f'Average rate: {avg_rate:.2f} Hz')
            self.get_logger().info(f'Min interval: {self.min_interval:.3f}s')
            self.get_logger().info(f'Max interval: {self.max_interval:.3f}s')


def main(args=None):
    rclpy.init(args=args)
    node = TopicStatisticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report_statistics()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
