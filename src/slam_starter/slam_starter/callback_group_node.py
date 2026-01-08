#!/usr/bin/env python3
"""
ROS2 Callback Group Node
Demonstrates using multiple callback groups
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import time


class CallbackGroupNode(Node):
    """Node with multiple callback groups"""

    def __init__(self):
        super().__init__('callback_group_node')
        
        # Create different callback groups
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()
        
        # Publishers
        self.pub1 = self.create_publisher(String, 'topic1', 10)
        self.pub2 = self.create_publisher(String, 'topic2', 10)
        
        # Timers in different callback groups
        self.timer1 = self.create_timer(
            1.0, self.callback1, callback_group=self.group1
        )
        self.timer2 = self.create_timer(
            1.5, self.callback2, callback_group=self.group2
        )
        self.timer3 = self.create_timer(
            0.5, self.callback3, callback_group=self.group3
        )
        
        self.get_logger().info('Callback Group Node started')
        self.get_logger().info('Group 1 & 2: Mutually Exclusive')
        self.get_logger().info('Group 3: Reentrant')

    def callback1(self):
        """Callback in group 1 - slow operation"""
        self.get_logger().info('Callback 1 START (Group 1, 2s delay)')
        time.sleep(2.0)
        msg = String()
        msg.data = 'Message from callback 1'
        self.pub1.publish(msg)
        self.get_logger().info('Callback 1 END')

    def callback2(self):
        """Callback in group 2 - can run parallel to group 1"""
        self.get_logger().info('Callback 2 (Group 2, instant)')
        msg = String()
        msg.data = 'Message from callback 2'
        self.pub2.publish(msg)

    def callback3(self):
        """Callback in group 3 - reentrant"""
        self.get_logger().info('Callback 3 (Group 3 - Reentrant)')


def main(args=None):
    rclpy.init(args=args)
    node = CallbackGroupNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
