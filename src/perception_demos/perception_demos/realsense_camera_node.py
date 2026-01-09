#!/usr/bin/env python3
"""
RealSense Camera Node - Interface with Intel RealSense depth cameras.

This node demonstrates:
- pyrealsense2 library integration
- Publishing RGB and depth images
- Aligned depth to color
- Point cloud generation
- Camera intrinsics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np


class RealSenseCameraNode(Node):
    """Publishes images and depth data from Intel RealSense camera."""

    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # Declare parameters
        self.declare_parameter('serial_number', '')
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_pointcloud', False)
        
        # Get parameters
        serial = self.get_parameter('serial_number').value
        fps = self.get_parameter('frame_rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        enable_depth = self.get_parameter('enable_depth').value
        enable_color = self.get_parameter('enable_color').value
        enable_pc = self.get_parameter('enable_pointcloud').value
        
        # Publishers
        if enable_color:
            self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
            self.color_info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 10)
        
        if enable_depth:
            self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
            self.depth_info_pub = self.create_publisher(CameraInfo, 'camera/depth/camera_info', 10)
        
        if enable_pc:
            self.pointcloud_pub = self.create_publisher(PointCloud2, 'camera/depth/points', 10)
        
        self.bridge = CvBridge()
        
        # Simulated pipeline (in real implementation, use pyrealsense2)
        self.get_logger().info(
            'RealSense camera node initialized (simulated mode)\n'
            f'  Resolution: {width}x{height}\n'
            f'  Frame rate: {fps} fps\n'
            f'  Color: {enable_color}, Depth: {enable_depth}, PointCloud: {enable_pc}'
        )
        
        # Note: Real implementation would use:
        # import pyrealsense2 as rs
        # self.pipeline = rs.pipeline()
        # config = rs.config()
        # if serial: config.enable_device(serial)
        # config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        # config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        # self.pipeline.start(config)
        
        # Timer for simulated data
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Publish camera data (simulated)."""
        timestamp = self.get_clock().now().to_msg()
        
        # In real implementation, get frames from RealSense:
        # frames = self.pipeline.wait_for_frames()
        # aligned_frames = self.align.process(frames)
        # color_frame = aligned_frames.get_color_frame()
        # depth_frame = aligned_frames.get_depth_frame()
        
        # Simulated color image
        color_image = np.zeros((480, 640, 3), dtype=np.uint8)
        color_image[:] = (100, 150, 200)  # Blue-ish background
        
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = timestamp
        color_msg.header.frame_id = 'camera_color_optical_frame'
        
        if hasattr(self, 'color_pub'):
            self.color_pub.publish(color_msg)
        
        # Simulated depth image
        depth_image = np.random.randint(500, 3000, (480, 640), dtype=np.uint16)
        
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        depth_msg.header.stamp = timestamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        
        if hasattr(self, 'depth_pub'):
            self.depth_pub.publish(depth_msg)
        
        self.get_logger().info('Published RealSense data (simulated)', once=True)

    def destroy_node(self):
        """Clean up resources."""
        # Real implementation: self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
