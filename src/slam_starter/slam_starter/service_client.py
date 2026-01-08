#!/usr/bin/env python3
"""
ROS2 Service Client Node
Calls the AddTwoInts service
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Service client that calls the AddTwoInts service"""

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        """Send a service request"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: service_client.py <int> <int>')
        return
    
    node = AddTwoIntsClient()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    rclpy.spin_until_future_complete(node, future)
    
    try:
        result = future.result()
        node.get_logger().info(f'Result: {result.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
