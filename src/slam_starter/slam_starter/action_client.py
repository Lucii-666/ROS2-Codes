#!/usr/bin/env python3
"""
ROS2 Action Client Node
Calls the Fibonacci action server
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """Action client that calls the Fibonacci action"""

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Fibonacci Action Client started')

    def send_goal(self, order):
        """Send a goal to the action server"""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: order={order}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)
    
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
