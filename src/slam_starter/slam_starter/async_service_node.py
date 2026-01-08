#!/usr/bin/env python3
"""
ROS2 Async Service Node
Demonstrates asynchronous service handling
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import asyncio


class AsyncServiceNode(Node):
    """Node with asynchronous service"""

    def __init__(self):
        super().__init__('async_service_node')
        
        self.srv = self.create_service(
            AddTwoInts,
            'async_add_two_ints',
            self.async_service_callback
        )
        
        self.get_logger().info('Async Service Node started')

    async def async_service_callback(self, request, response):
        """Asynchronous service callback"""
        self.get_logger().info(f'Async request: {request.a} + {request.b}')
        
        # Simulate async operation
        await asyncio.sleep(1.0)
        
        response.sum = request.a + request.b
        self.get_logger().info(f'Async response: {response.sum}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AsyncServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
