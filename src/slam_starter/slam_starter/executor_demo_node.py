#!/usr/bin/env python3
"""
ROS2 Custom Executor Node
Demonstrates custom executor configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from std_msgs.msg import String


class ExecutorDemoNode(Node):
    """Node for executor demonstration"""

    def __init__(self):
        super().__init__('executor_demo_node')
        
        self.publisher = self.create_publisher(String, 'executor_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'executor_topic',
            self.callback,
            10
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Executor Demo Node started')

    def timer_callback(self):
        """Publish messages"""
        msg = String()
        msg.data = f'Executor message #{self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

    def callback(self, msg):
        """Receive messages"""
        self.get_logger().info(f'Received: {msg.data}')


def run_single_threaded():
    """Run with single-threaded executor"""
    rclpy.init()
    node = ExecutorDemoNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    node.get_logger().info('Using SingleThreadedExecutor')
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def run_multi_threaded():
    """Run with multi-threaded executor"""
    rclpy.init()
    node = ExecutorDemoNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    node.get_logger().info('Using MultiThreadedExecutor (2 threads)')
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Default to multi-threaded"""
    run_multi_threaded()


if __name__ == '__main__':
    main()
