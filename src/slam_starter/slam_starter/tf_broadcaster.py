#!/usr/bin/env python3
"""
ROS2 TF Broadcaster Node
Broadcasts transforms to the TF tree
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TFBroadcaster(Node):
    """Node that broadcasts transforms"""

    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Create a transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a timer to broadcast transforms periodically
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        
        self.angle = 0.0
        
        self.get_logger().info('TF Broadcaster Node started')

    def broadcast_timer_callback(self):
        """Broadcast a transform"""
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'
        
        # Translation - circular motion
        radius = 2.0
        t.transform.translation.x = radius * math.cos(self.angle)
        t.transform.translation.y = radius * math.sin(self.angle)
        t.transform.translation.z = 0.0
        
        # Rotation - rotating around z-axis
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.angle / 2.0)
        t.transform.rotation.w = math.cos(self.angle / 2.0)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Increment angle
        self.angle += 0.05
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
