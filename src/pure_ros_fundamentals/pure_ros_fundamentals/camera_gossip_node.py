#!/usr/bin/env python3
"""
Camera Gossip Node
The Camera that Gossips - sees color and shape, whispers to everyone.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np


class CameraGossipNode(Node):
    """
    Cameras are gossips. They see everything and tell everyone.
    RGB values. Depth. Pixels. No judgment. Just raw data.
    """

    def __init__(self):
        super().__init__('camera_gossip')
        
        # Publishers - multiple pigeons for different messages
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Timer - gossip happens 30 times per second
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)
        
        # Camera properties
        self.width = 640
        self.height = 480
        self.frame_count = 0
        
        self.get_logger().info('📷 Camera Gossip online. I see... everything.')

    def publish_frame(self):
        """
        Every frame, the camera captures light and converts it to numbers.
        Pure sensor data - just counting photons.
        """
        # Create a simple synthetic image (in real world, this comes from hardware)
        # Let's make a gradient that changes over time
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Create a moving color pattern
        phase = (self.frame_count % 255) / 255.0
        for y in range(self.height):
            for x in range(self.width):
                img[y, x, 0] = int(255 * (x / self.width) * phase)  # Red channel
                img[y, x, 1] = int(255 * (y / self.height))  # Green channel
                img[y, x, 2] = int(255 * (1.0 - phase))  # Blue channel
        
        # Publish image
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = self.width * 3
        img_msg.data = img.flatten().tolist()
        
        self.image_pub.publish(img_msg)
        
        # Publish camera info (intrinsic parameters)
        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.height = self.height
        info_msg.width = self.width
        
        # Simulated camera matrix (focal length, principal point)
        fx = 500.0
        fy = 500.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        
        self.info_pub.publish(info_msg)
        
        self.frame_count += 1
        
        if self.frame_count % 30 == 0:
            self.get_logger().debug(f'📷 Gossiped {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = CameraGossipNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('📷 Camera stops gossiping.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
