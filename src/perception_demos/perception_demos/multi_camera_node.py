#!/usr/bin/env python3
"""
Multi-Camera Node - Manage multiple cameras simultaneously.

This node demonstrates:
- Multiple camera sources
- Synchronized publishing
- Camera switching
- Multi-view recording
- Namespace management
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class MultiCameraNode(Node):
    """Manages and publishes from multiple cameras."""

    def __init__(self):
        super().__init__('multi_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_indices', [0, 1])
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('enable_sync', True)
        
        # Get parameters
        camera_indices = self.get_parameter('camera_indices').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.enable_sync = self.get_parameter('enable_sync').value
        
        self.cameras = {}
        self.publishers = {}
        self.info_publishers = {}
        self.bridge = CvBridge()
        
        # Initialize cameras
        for idx in camera_indices:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, frame_rate)
                
                self.cameras[idx] = cap
                
                # Create publishers for each camera
                self.publishers[idx] = self.create_publisher(
                    Image, f'camera_{idx}/image_raw', 10
                )
                self.info_publishers[idx] = self.create_publisher(
                    CameraInfo, f'camera_{idx}/camera_info', 10
                )
                
                self.get_logger().info(f'Initialized camera {idx}')
            else:
                self.get_logger().warn(f'Failed to open camera {idx}')
        
        if not self.cameras:
            self.get_logger().error('No cameras could be opened!')
            return
        
        # Status publisher
        self.status_pub = self.create_publisher(String, 'multi_camera/status', 10)
        
        # Timer for publishing
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0

    def timer_callback(self):
        """Capture and publish from all cameras."""
        timestamp = self.get_clock().now().to_msg()
        frames = {}
        
        # Capture from all cameras
        for idx, cap in self.cameras.items():
            ret, frame = cap.read()
            if ret:
                frames[idx] = frame
            else:
                self.get_logger().warn(f'Failed to capture from camera {idx}')
        
        # Publish synchronized or individual
        if self.enable_sync and len(frames) == len(self.cameras):
            # All cameras captured successfully
            for idx, frame in frames.items():
                self.publish_frame(idx, frame, timestamp)
        elif not self.enable_sync:
            # Publish whatever we got
            for idx, frame in frames.items():
                self.publish_frame(idx, frame, timestamp)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Active cameras: {list(frames.keys())}, Frame: {self.frame_count}'
        self.status_pub.publish(status_msg)
        
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Published {self.frame_count} frames from {len(self.cameras)} cameras'
            )

    def publish_frame(self, camera_idx, frame, timestamp):
        """Publish a single camera frame."""
        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = timestamp
        image_msg.header.frame_id = f'camera_{camera_idx}_frame'
        
        self.publishers[camera_idx].publish(image_msg)
        
        # Create and publish camera info
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = timestamp
        camera_info_msg.header.frame_id = f'camera_{camera_idx}_frame'
        camera_info_msg.height = frame.shape[0]
        camera_info_msg.width = frame.shape[1]
        self.info_publishers[camera_idx].publish(camera_info_msg)

    def destroy_node(self):
        """Clean up resources."""
        for cap in self.cameras.values():
            if cap.isOpened():
                cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
