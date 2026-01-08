#!/usr/bin/env python3
"""
ROS2 Multi-threaded Executor Node
Demonstrates using multi-threaded executor for parallel callbacks
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
import time


class MultiThreadedNode(Node):
    """Node using multi-threaded executor"""

    def __init__(self):
        super().__init__('multi_threaded_node')
        
        # Create callback groups
        self.reentrant_group = ReentrantCallbackGroup()
        self.exclusive_group = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.publisher = self.create_publisher(String, 'output', 10)
        
        # Create timers in different callback groups
        self.timer1 = self.create_timer(
            1.0, 
            self.slow_callback,
            callback_group=self.reentrant_group
        )
        
        self.timer2 = self.create_timer(
            0.5, 
            self.fast_callback,
            callback_group=self.reentrant_group
        )
        
        self.timer3 = self.create_timer(
            2.0,
            self.exclusive_callback,
            callback_group=self.exclusive_group
        )
        
        self.get_logger().info('Multi-threaded Node started')
        self.get_logger().info('Timer 1 (slow): 1 Hz - Reentrant group')
        self.get_logger().info('Timer 2 (fast): 2 Hz - Reentrant group')
        self.get_logger().info('Timer 3 (exclusive): 0.5 Hz - Exclusive group')

    def slow_callback(self):
        """Slow callback that takes time"""
        self.get_logger().info('Slow callback START (takes 2s)')
        time.sleep(2.0)  # Simulate slow processing
        msg = String()
        msg.data = 'Slow callback completed'
        self.publisher.publish(msg)
        self.get_logger().info('Slow callback END')

    def fast_callback(self):
        """Fast callback"""
        self.get_logger().info('Fast callback (instant)')
        msg = String()
        msg.data = 'Fast callback'
        self.publisher.publish(msg)

    def exclusive_callback(self):
        """Exclusive callback"""
        self.get_logger().info('Exclusive callback START')
        time.sleep(1.0)
        msg = String()
        msg.data = 'Exclusive callback'
        self.publisher.publish(msg)
        self.get_logger().info('Exclusive callback END')


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiThreadedNode()
    
    # Use MultiThreadedExecutor with 4 threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
