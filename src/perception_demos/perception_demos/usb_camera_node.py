#!/usr/bin/env python3
"""
USB Camera Node - Capture and publish images from USB webcam.

This node demonstrates:
- OpenCV VideoCapture for USB cameras
- cv_bridge for ROS2-OpenCV conversion
- Publishing sensor_msgs/Image
- Camera info publishing
- Dynamic parameter configuration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class USBCameraNode(Node):
    """Publishes images from a USB webcam."""

    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'camera_frame')
        
        # Get parameters
        camera_index = self.get_parameter('camera_index').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_index}')
            return
        
        self.get_logger().info(f'Opened USB camera {camera_index} at {width}x{height}@{frame_rate}fps')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Timer for publishing
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0

    def timer_callback(self):
        """Capture and publish camera frame."""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Convert to ROS Image message
        timestamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = timestamp
        image_msg.header.frame_id = self.frame_id
        
        # Publish image
        self.image_pub.publish(image_msg)
        
        # Create and publish camera info
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = timestamp
        camera_info_msg.header.frame_id = self.frame_id
        camera_info_msg.height = frame.shape[0]
        camera_info_msg.width = frame.shape[1]
        self.camera_info_pub.publish(camera_info_msg)
        
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
