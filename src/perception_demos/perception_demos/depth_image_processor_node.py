#!/usr/bin/env python3
"""Depth Image Processor Node - Process and analyze depth images."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthImageProcessorNode(Node):
    def __init__(self):
        super().__init__('depth_image_processor_node')
        
        self.declare_parameter('min_depth', 0.3)  # meters
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('apply_colormap', True)
        self.declare_parameter('filter_invalid', True)
        
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.apply_colormap = self.get_parameter('apply_colormap').value
        self.filter_invalid = self.get_parameter('filter_invalid').value
        
        self.bridge = CvBridge()
        
        self.depth_sub = self.create_subscription(Image, 'camera/depth/image_raw',
                                                  self.depth_callback, 10)
        self.processed_pub = self.create_publisher(Image, 'depth/processed', 10)
        self.visualization_pub = self.create_publisher(Image, 'depth/visualization', 10)
        
        self.get_logger().info(f'Depth processor started (range: {self.min_depth}-{self.max_depth}m)')

    def depth_callback(self, msg):
        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Filter by depth range
            depth_filtered = depth_image.copy()
            depth_filtered[depth_filtered < self.min_depth] = 0
            depth_filtered[depth_filtered > self.max_depth] = 0
            
            # Normalize for visualization
            depth_normalized = cv2.normalize(depth_filtered, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = depth_normalized.astype(np.uint8)
            
            # Apply colormap
            if self.apply_colormap:
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            else:
                depth_colored = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
            
            # Add depth info overlay
            mean_depth = np.mean(depth_filtered[depth_filtered > 0])
            cv2.putText(depth_colored, f'Mean Depth: {mean_depth:.2f}m', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish
            processed_msg = self.bridge.cv2_to_imgmsg(depth_filtered.astype(np.float32))
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)
            
            vis_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
            vis_msg.header = msg.header
            self.visualization_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
