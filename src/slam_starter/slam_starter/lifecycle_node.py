#!/usr/bin/env python3
"""
ROS2 Lifecycle Node
Demonstrates lifecycle node management
"""

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String


class LifecycleNode(Node):
    """Managed lifecycle node"""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.publisher_ = None
        self.timer = None
        self.counter = 0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node"""
        self.get_logger().info('on_configure() is called')
        self.publisher_ = self.create_lifecycle_publisher(String, 'lifecycle_chatter', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node"""
        self.get_logger().info('on_activate() is called')
        self.timer = self.create_timer(1.0, self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node"""
        self.get_logger().info('on_deactivate() is called')
        if self.timer:
            self.destroy_timer(self.timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup the node"""
        self.get_logger().info('on_cleanup() is called')
        if self.publisher_:
            self.destroy_publisher(self.publisher_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node"""
        self.get_logger().info('on_shutdown() is called')
        if self.timer:
            self.destroy_timer(self.timer)
        if self.publisher_:
            self.destroy_publisher(self.publisher_)
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Timer callback to publish messages"""
        if self.publisher_ and self.publisher_.is_activated:
            msg = String()
            msg.data = f'Lifecycle message #{self.counter}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = LifecycleNode('lifecycle_node')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
