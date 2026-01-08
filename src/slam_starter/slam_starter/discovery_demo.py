#!/usr/bin/env python3
"""
ROS2 Node Discovery Demo
Demonstrates node introspection and discovery
"""

import rclpy
from rclpy.node import Node


class DiscoveryDemo(Node):
    """Node demonstrating discovery and introspection"""

    def __init__(self):
        super().__init__('discovery_demo')
        
        self.timer = self.create_timer(3.0, self.discover_nodes)
        
        self.get_logger().info('Discovery Demo Node started')

    def discover_nodes(self):
        """Discover and list all nodes in the system"""
        self.get_logger().info('=== Discovering Nodes ===')
        
        # Get all node names
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        
        self.get_logger().info(f'Found {len(node_names_and_namespaces)} nodes:')
        
        for name, namespace in node_names_and_namespaces:
            full_name = f'{namespace}/{name}' if namespace != '/' else f'/{name}'
            self.get_logger().info(f'  - {full_name}')
            
            # Get topics for each node
            try:
                topic_names_and_types = self.get_publisher_names_and_types_by_node(name, namespace)
                if topic_names_and_types:
                    self.get_logger().info(f'    Publishers:')
                    for topic_name, topic_types in topic_names_and_types:
                        self.get_logger().info(f'      * {topic_name} ({", ".join(topic_types)})')
                
                subscriber_names_and_types = self.get_subscriber_names_and_types_by_node(name, namespace)
                if subscriber_names_and_types:
                    self.get_logger().info(f'    Subscribers:')
                    for topic_name, topic_types in subscriber_names_and_types:
                        self.get_logger().info(f'      * {topic_name} ({", ".join(topic_types)})')
            except Exception as e:
                self.get_logger().warn(f'    Could not get details: {e}')
        
        self.get_logger().info('===')


def main(args=None):
    rclpy.init(args=args)
    node = DiscoveryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
