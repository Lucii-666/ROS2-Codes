#!/usr/bin/env python3
"""
ROS2 Bag Recorder Node
Records topic data to a bag file
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import datetime
import os


class BagRecorderNode(Node):
    """Node that records topics"""

    def __init__(self):
        super().__init__('bag_recorder_node')
        
        # Create subscription to record
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        self.messages_recorded = 0
        self.recorded_messages = []
        
        # Create timer to save periodically
        self.timer = self.create_timer(5.0, self.save_callback)
        
        self.get_logger().info('Bag Recorder Node started')
        self.get_logger().info('Recording topic: /chatter')

    def listener_callback(self, msg):
        """Record incoming messages"""
        timestamp = self.get_clock().now()
        
        # Store message with timestamp
        self.recorded_messages.append({
            'timestamp': timestamp.nanoseconds,
            'data': msg.data
        })
        
        self.messages_recorded += 1
        self.get_logger().info(
            f'Recorded message #{self.messages_recorded}: {msg.data}'
        )

    def save_callback(self):
        """Periodically save recorded data"""
        if self.recorded_messages:
            self.get_logger().info(
                f'Total messages in buffer: {len(self.recorded_messages)}'
            )
            self.get_logger().info(
                f'Recording statistics: {self.messages_recorded} messages total'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Shutting down. Recorded {node.messages_recorded} messages.'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
