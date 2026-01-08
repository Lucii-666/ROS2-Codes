#!/usr/bin/env python3
"""
ROS2 Twist Publisher Node
Publishes velocity commands (Twist messages)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class TwistPublisher(Node):
    """Node that publishes velocity commands"""

    def __init__(self):
        super().__init__('twist_publisher')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.angle = 0.0
        
        self.get_logger().info('Twist Publisher Node started')
        self.get_logger().info('Publishing circular motion commands to /cmd_vel')

    def timer_callback(self):
        """Publish twist messages for circular motion"""
        msg = Twist()
        
        # Linear velocity (forward)
        msg.linear.x = 0.5  # 0.5 m/s forward
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Angular velocity (rotation around z-axis)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5 * math.sin(self.angle)  # Varying angular velocity
        
        self.publisher.publish(msg)
        
        if int(self.angle * 10) % 10 == 0:  # Log every ~1 second
            self.get_logger().info(
                f'Publishing: linear.x={msg.linear.x:.2f}, '
                f'angular.z={msg.angular.z:.2f}'
            )
        
        self.angle += 0.1
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi


def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop command
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.get_logger().info('Sent stop command')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
