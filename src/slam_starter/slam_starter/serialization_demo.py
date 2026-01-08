#!/usr/bin/env python3
"""
ROS2 Serialization Demo Node
Demonstrates message serialization and deserialization
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import String


class SerializationDemo(Node):
    """Node demonstrating message serialization"""

    def __init__(self):
        super().__init__('serialization_demo')
        
        self.timer = self.create_timer(2.0, self.demo_serialization)
        self.counter = 0
        
        self.get_logger().info('Serialization Demo Node started')

    def demo_serialization(self):
        """Demonstrate serialization/deserialization"""
        # Create a message
        original_msg = String()
        original_msg.data = f'Serialization test #{self.counter}'
        
        self.get_logger().info(f'Original message: {original_msg.data}')
        
        # Serialize the message
        serialized_data = serialize_message(original_msg)
        self.get_logger().info(f'Serialized to {len(serialized_data)} bytes')
        self.get_logger().info(f'Serialized data (hex): {serialized_data.hex()[:50]}...')
        
        # Deserialize the message
        deserialized_msg = deserialize_message(serialized_data, String)
        self.get_logger().info(f'Deserialized message: {deserialized_msg.data}')
        
        # Verify
        if original_msg.data == deserialized_msg.data:
            self.get_logger().info('✓ Serialization/deserialization successful!')
        else:
            self.get_logger().error('✗ Serialization/deserialization failed!')
        
        self.counter += 1
        self.get_logger().info('---')


def main(args=None):
    rclpy.init(args=args)
    node = SerializationDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
