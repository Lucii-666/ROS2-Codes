#!/usr/bin/env python3
"""
ROS2 Parameter Node
Demonstrates parameter declaration and handling
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


class ParameterNode(Node):
    """Node that demonstrates parameter handling"""

    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with descriptors
        self.declare_parameter(
            'my_string',
            'Hello',
            ParameterDescriptor(description='A string parameter')
        )
        self.declare_parameter(
            'my_int',
            42,
            ParameterDescriptor(description='An integer parameter')
        )
        self.declare_parameter(
            'my_float',
            3.14,
            ParameterDescriptor(description='A float parameter')
        )
        self.declare_parameter(
            'my_bool',
            True,
            ParameterDescriptor(description='A boolean parameter')
        )
        
        # Create a timer to periodically read and log parameters
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Parameter Node started')
        self.log_parameters()

    def log_parameters(self):
        """Log all parameter values"""
        my_string = self.get_parameter('my_string').value
        my_int = self.get_parameter('my_int').value
        my_float = self.get_parameter('my_float').value
        my_bool = self.get_parameter('my_bool').value
        
        self.get_logger().info(f'my_string: {my_string}')
        self.get_logger().info(f'my_int: {my_int}')
        self.get_logger().info(f'my_float: {my_float}')
        self.get_logger().info(f'my_bool: {my_bool}')

    def timer_callback(self):
        """Periodic timer callback"""
        self.get_logger().info('--- Current Parameter Values ---')
        self.log_parameters()


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
