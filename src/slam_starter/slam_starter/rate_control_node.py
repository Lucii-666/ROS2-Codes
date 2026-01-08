#!/usr/bin/env python3
"""
ROS2 Rate Control Node
Demonstrates rate-based publishing control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RateControlNode(Node):
    """Node demonstrating rate control"""

    def __init__(self):
        super().__init__('rate_control_node')
        
        # Create publishers with different strategies
        self.publisher = self.create_publisher(String, 'rate_controlled', 10)
        
        # Use timer for rate control (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.counter = 0
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Rate Control Node started at 10 Hz')

    def timer_callback(self):
        """Publish at controlled rate"""
        msg = String()
        msg.data = f'Rate-controlled message #{self.counter}'
        
        # Calculate actual rate
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed > 0:
            actual_rate = self.counter / elapsed
        else:
            actual_rate = 0.0
        
        self.publisher.publish(msg)
        
        if self.counter % 10 == 0:
            self.get_logger().info(
                f'Published {self.counter} messages, '
                f'Actual rate: {actual_rate:.2f} Hz'
            )
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = RateControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Calculate final statistics
        elapsed = (node.get_clock().now() - node.start_time).nanoseconds / 1e9
        final_rate = node.counter / elapsed if elapsed > 0 else 0
        node.get_logger().info(
            f'Final statistics: {node.counter} messages in {elapsed:.2f}s, '
            f'Average rate: {final_rate:.2f} Hz'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
