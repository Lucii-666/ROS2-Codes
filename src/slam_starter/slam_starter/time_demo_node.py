#!/usr/bin/env python3
"""
ROS2 Time Demo Node
Demonstrates ROS time usage and manipulation
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from builtin_interfaces.msg import Time


class TimeDemo(Node):
    """Node demonstrating ROS time"""

    def __init__(self):
        super().__init__('time_demo')
        
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Time Demo Node started')

    def timer_callback(self):
        """Demonstrate time operations"""
        # Get current time
        current_time = self.get_clock().now()
        
        # Calculate elapsed time
        elapsed = current_time - self.start_time
        elapsed_sec = elapsed.nanoseconds / 1e9
        
        # Create duration
        duration_5s = Duration(seconds=5)
        
        # Time arithmetic
        future_time = current_time + duration_5s
        
        self.get_logger().info('--- Time Information ---')
        self.get_logger().info(f'Current time: {current_time.nanoseconds}ns')
        self.get_logger().info(f'Elapsed: {elapsed_sec:.2f}s')
        self.get_logger().info(f'Future time (+5s): {future_time.nanoseconds}ns')
        
        # Convert to seconds
        current_sec = current_time.nanoseconds / 1e9
        self.get_logger().info(f'Current time in seconds: {current_sec:.3f}s')


def main(args=None):
    rclpy.init(args=args)
    node = TimeDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
