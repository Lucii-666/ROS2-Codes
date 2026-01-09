#!/usr/bin/env python3
"""
Image Filter Node - Apply various OpenCV filters to images.

This node demonstrates:
- Multiple filter types (Gaussian, Median, Bilateral, Morphological)
- Dynamic reconfiguration
- Filter chaining
- Performance comparison
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageFilterNode(Node):
    """Applies various image filters."""

    def __init__(self):
        super().__init__('image_filter_node')
        
        # Declare parameters
        self.declare_parameter('filter_type', 'gaussian')  # gaussian, median, bilateral, box, morphology
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('sigma', 1.0)
        self.declare_parameter('morph_operation', 'open')  # open, close, gradient, tophat, blackhat
        self.declare_parameter('bilateral_d', 9)
        self.declare_parameter('bilateral_sigma_color', 75)
        self.declare_parameter('bilateral_sigma_space', 75)
        
        # Get parameters
        self.filter_type = self.get_parameter('filter_type').value
        self.kernel_size = self.get_parameter('kernel_size').value
        self.sigma = self.get_parameter('sigma').value
        self.morph_op = self.get_parameter('morph_operation').value
        self.bilateral_d = self.get_parameter('bilateral_d').value
        self.bilateral_sigma_color = self.get_parameter('bilateral_sigma_color').value
        self.bilateral_sigma_space = self.get_parameter('bilateral_sigma_space').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.filtered_pub = self.create_publisher(Image, 'image_filtered', 10)
        self.comparison_pub = self.create_publisher(Image, 'image_comparison', 10)
        self.status_pub = self.create_publisher(String, 'filter_status', 10)
        
        self.get_logger().info(
            f'Image filter node started\n'
            f'  Filter: {self.filter_type}\n'
            f'  Kernel size: {self.kernel_size}'
        )

    def image_callback(self, msg):
        """Apply filter to incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Apply filter based on type
            if self.filter_type == 'gaussian':
                filtered = cv2.GaussianBlur(
                    cv_image, (self.kernel_size, self.kernel_size), self.sigma
                )
            elif self.filter_type == 'median':
                filtered = cv2.medianBlur(cv_image, self.kernel_size)
            elif self.filter_type == 'bilateral':
                filtered = cv2.bilateralFilter(
                    cv_image, self.bilateral_d,
                    self.bilateral_sigma_color, self.bilateral_sigma_space
                )
            elif self.filter_type == 'box':
                filtered = cv2.boxFilter(cv_image, -1, (self.kernel_size, self.kernel_size))
            elif self.filter_type == 'morphology':
                filtered = self.apply_morphology(cv_image)
            else:
                filtered = cv_image.copy()
            
            # Publish filtered image
            filtered_msg = self.bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
            filtered_msg.header = msg.header
            self.filtered_pub.publish(filtered_msg)
            
            # Create comparison image
            comparison = np.hstack([cv_image, filtered])
            comparison_msg = self.bridge.cv2_to_imgmsg(comparison, encoding='bgr8')
            comparison_msg.header = msg.header
            self.comparison_pub.publish(comparison_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = f'Filter: {self.filter_type}, Kernel: {self.kernel_size}'
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error applying filter: {e}')

    def apply_morphology(self, image):
        """Apply morphological operations."""
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size)
        )
        
        if self.morph_op == 'open':
            return cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        elif self.morph_op == 'close':
            return cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        elif self.morph_op == 'gradient':
            return cv2.morphologyEx(image, cv2.MORPH_GRADIENT, kernel)
        elif self.morph_op == 'tophat':
            return cv2.morphologyEx(image, cv2.MORPH_TOPHAT, kernel)
        elif self.morph_op == 'blackhat':
            return cv2.morphologyEx(image, cv2.MORPH_BLACKHAT, kernel)
        else:
            return image


def main(args=None):
    rclpy.init(args=args)
    node = ImageFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
