#!/usr/bin/env python3
"""
TF Broadcaster Node
The Coordinate Frame Oracle - maintains the tree of transforms.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class TFBroadcasterNode(Node):
    """
    TF is the backbone of ROS.
    Every sensor, every joint, every link has a coordinate frame.
    This node broadcasts the relationships between them all.
    
    The TF tree is like a family tree, but for robot parts.
    """

    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer - broadcast transforms at 50 Hz
        self.timer = self.create_timer(0.02, self.broadcast_frames)
        
        self.angle = 0.0
        
        self.get_logger().info('🌳 TF Tree Broadcaster active. Building coordinate frames...')

    def broadcast_frames(self):
        """
        Broadcast the tree of transforms.
        This is how ROS knows where everything is relative to everything else.
        """
        now = self.get_clock().now().to_msg()
        
        # Transform 1: map -> odom
        # (SLAM updates this as the robot moves)
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0
        
        # Transform 2: odom -> base_link
        # (Odometry updates this from wheel encoders)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        # Transform 3: base_link -> laser_frame
        # (Fixed relationship - defined in URDF)
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'laser_frame'
        t3.transform.translation.x = 0.1  # 10cm in front of robot
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.15  # 15cm up
        t3.transform.rotation.w = 1.0
        
        # Transform 4: base_link -> camera_frame
        # (Another fixed relationship)
        t4 = TransformStamped()
        t4.header.stamp = now
        t4.header.frame_id = 'base_link'
        t4.child_frame_id = 'camera_frame'
        t4.transform.translation.x = 0.15  # 15cm in front
        t4.transform.translation.y = 0.0
        t4.transform.translation.z = 0.2  # 20cm up
        
        # Camera tilted down 15 degrees
        tilt_angle = -15.0 * math.pi / 180.0
        t4.transform.rotation.x = math.sin(tilt_angle / 2.0)
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = math.cos(tilt_angle / 2.0)
        
        # Broadcast all transforms
        self.tf_broadcaster.sendTransform([t1, t2, t3, t4])
        
        self.angle += 0.01
        
        if int(self.angle * 100) % 100 == 0:
            self.get_logger().debug(
                '🌳 TF Tree:\n'
                '   map\n'
                '    └─ odom\n'
                '        └─ base_link\n'
                '            ├─ laser_frame\n'
                '            └─ camera_frame'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🌳 TF Tree collapsed.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
