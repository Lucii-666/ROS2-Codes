#!/usr/bin/env python3
"""
ROS2 Timer Callback Node
Demonstrates timer-based callbacks with multiple timers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimerCallbackNode(Node):
    """Node with multiple timer callbacks"""

    def __init__(self):
        super().__init__('timer_callback_node')
        
        # Create multiple timers with different frequencies
        self.timer1 = self.create_timer(1.0, self.timer1_callback)
        self.timer2 = self.create_timer(2.5, self.timer2_callback)
        self.timer3 = self.create_timer(5.0, self.timer3_callback)
        
        self.counter1 = 0
        self.counter2 = 0
        self.counter3 = 0
        
        self.publisher = self.create_publisher(String, 'timer_output', 10)
        
        self.get_logger().info('Timer Callback Node started with 3 timers')
        self.get_logger().info('Timer 1: 1.0 Hz, Timer 2: 0.4 Hz, Timer 3: 0.2 Hz')

    def timer1_callback(self):
        """Fast timer - 1 Hz"""
        self.counter1 += 1
        msg = String()
        msg.data = f'Timer 1 (1 Hz): tick {self.counter1}'
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)

    def timer2_callback(self):
        """Medium timer - 0.4 Hz (every 2.5 seconds)"""
        self.counter2 += 1
        msg = String()
        msg.data = f'Timer 2 (0.4 Hz): tick {self.counter2}'
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)

    def timer3_callback(self):
        """Slow timer - 0.2 Hz (every 5 seconds)"""
        self.counter3 += 1
        msg = String()
        msg.data = f'Timer 3 (0.2 Hz): tick {self.counter3}'
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TimerCallbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
