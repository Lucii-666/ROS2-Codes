#!/usr/bin/env python3
"""
ROS2 Logger Demo Node
Demonstrates different logging levels in ROS2
"""

import rclpy
from rclpy.node import Node


class LoggerDemo(Node):
    """Node demonstrating logging levels"""

    def __init__(self):
        super().__init__('logger_demo')
        
        # Create timer to demonstrate logging
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Logger Demo Node started')
        self.log_all_levels()

    def log_all_levels(self):
        """Demonstrate all logging levels"""
        self.get_logger().debug('This is a DEBUG message (most verbose)')
        self.get_logger().info('This is an INFO message (normal operation)')
        self.get_logger().warn('This is a WARN message (something unexpected)')
        self.get_logger().error('This is an ERROR message (something failed)')
        self.get_logger().fatal('This is a FATAL message (critical failure)')

    def timer_callback(self):
        """Periodic logging demonstration"""
        self.counter += 1
        
        self.get_logger().info(f'--- Logging cycle #{self.counter} ---')
        
        # Log at different levels based on counter
        if self.counter % 5 == 0:
            self.get_logger().warn(f'Counter reached {self.counter} (divisible by 5)')
        
        if self.counter % 10 == 0:
            self.get_logger().error(f'Counter reached {self.counter} (divisible by 10)')
        
        # Always log debug and info
        self.get_logger().debug(f'Debug: Current counter value is {self.counter}')
        self.get_logger().info(f'Info: Timer callback #{self.counter}')
        
        # Use formatted logging
        self.get_logger().info(
            f'Node: {self.get_name()}, '
            f'Namespace: {self.get_namespace()}, '
            f'Count: {self.counter}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LoggerDemo()
    
    # Set logging level (uncomment to change)
    # node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
