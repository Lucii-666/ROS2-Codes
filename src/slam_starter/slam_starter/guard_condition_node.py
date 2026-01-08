#!/usr/bin/env python3
"""
ROS2 Guard Condition Node
Demonstrates guard conditions for custom event handling
"""

import rclpy
from rclpy.node import Node
from rclpy.guard_condition import GuardCondition
import threading
import time


class GuardConditionNode(Node):
    """Node demonstrating guard conditions"""

    def __init__(self):
        super().__init__('guard_condition_node')
        
        # Create a guard condition
        self.guard_condition = GuardCondition()
        
        # Counter for triggered events
        self.trigger_count = 0
        
        # Start a thread that will trigger the guard condition
        self.trigger_thread = threading.Thread(target=self.trigger_loop)
        self.trigger_thread.daemon = True
        self.trigger_thread.start()
        
        self.get_logger().info('Guard Condition Node started')
        self.get_logger().info('Guard condition will be triggered every 3 seconds')

    def trigger_loop(self):
        """Background thread that triggers the guard condition"""
        while rclpy.ok():
            time.sleep(3.0)
            self.get_logger().info('Triggering guard condition from background thread')
            self.guard_condition.trigger()

    def handle_guard_condition(self):
        """Handle the guard condition trigger"""
        self.trigger_count += 1
        self.get_logger().info(
            f'Guard condition triggered! Count: {self.trigger_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GuardConditionNode()
    
    # Create executor and add node and guard condition
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Wait for guard condition triggers
    try:
        while rclpy.ok():
            # Wait for events (including guard condition)
            executor.spin_once(timeout_sec=0.1)
            
            # Check if guard condition was triggered
            if node.guard_condition.check_and_clear_trigger():
                node.handle_guard_condition()
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
