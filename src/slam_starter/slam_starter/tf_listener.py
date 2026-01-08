#!/usr/bin/env python3
"""
ROS2 TF Listener Node
Listens to transforms from the TF tree
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException


class TFListener(Node):
    """Node that listens to transforms"""

    def __init__(self):
        super().__init__('tf_listener')
        
        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a timer to query transforms periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('TF Listener Node started')

    def timer_callback(self):
        """Query and display transforms"""
        # Try to get transform from world to robot
        from_frame = 'world'
        to_frame = 'robot'
        
        try:
            # Look up the transform
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time()
            )
            
            # Log the transform
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            self.get_logger().info(
                f'Transform {from_frame} -> {to_frame}:\n'
                f'  Translation: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}\n'
                f'  Rotation: x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}'
            )
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {from_frame} to {to_frame}: {ex}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
