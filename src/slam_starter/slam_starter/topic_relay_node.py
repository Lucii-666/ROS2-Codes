#!/usr/bin/env python3
"""
ROS2 Topic Relay Node
Subscribes to one topic and republishes with transformation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopicRelayNode(Node):
    """Node that relays and transforms topic data"""

    def __init__(self):
        super().__init__('topic_relay_node')
        
        # Subscribe to input topic
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.relay_callback,
            10
        )
        
        # Publish to output topic
        self.publisher = self.create_publisher(String, 'output_topic', 10)
        
        self.message_count = 0
        
        self.get_logger().info('Topic Relay Node started')
        self.get_logger().info('Subscribing to: /input_topic')
        self.get_logger().info('Publishing to: /output_topic')

    def relay_callback(self, msg):
        """Relay and transform the message"""
        self.message_count += 1
        
        # Transform the message
        output_msg = String()
        output_msg.data = f'[RELAYED #{self.message_count}] {msg.data.upper()}'
        
        # Publish transformed message
        self.publisher.publish(output_msg)
        
        self.get_logger().info(f'Relayed: "{msg.data}" -> "{output_msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
