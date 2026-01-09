#!/usr/bin/env python3
"""
Edge Detection Node - Detect edges using various algorithms.

This node demonstrates:
- Canny edge detection
- Sobel operators
- Laplacian edge detection
- Scharr operator
- Adaptive thresholding
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class EdgeDetectionNode(Node):
    """Performs edge detection on images."""

    def __init__(self):
        super().__init__('edge_detection_node')
        
        # Declare parameters
        self.declare_parameter('algorithm', 'canny')  # canny, sobel, laplacian, scharr
        self.declare_parameter('canny_threshold1', 50)
        self.declare_parameter('canny_threshold2', 150)
        self.declare_parameter('sobel_kernel_size', 3)
        self.declare_parameter('sobel_scale', 1)
        self.declare_parameter('sobel_delta', 0)
        self.declare_parameter('apply_blur', True)
        self.declare_parameter('blur_kernel_size', 5)
        
        # Get parameters
        self.algorithm = self.get_parameter('algorithm').value
        self.canny_t1 = self.get_parameter('canny_threshold1').value
        self.canny_t2 = self.get_parameter('canny_threshold2').value
        self.sobel_ksize = self.get_parameter('sobel_kernel_size').value
        self.sobel_scale = self.get_parameter('sobel_scale').value
        self.sobel_delta = self.get_parameter('sobel_delta').value
        self.apply_blur = self.get_parameter('apply_blur').value
        self.blur_ksize = self.get_parameter('blur_kernel_size').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.edges_pub = self.create_publisher(Image, 'edges/image', 10)
        self.overlay_pub = self.create_publisher(Image, 'edges/overlay', 10)
        self.visualization_pub = self.create_publisher(Image, 'edges/visualization', 10)
        
        self.get_logger().info(
            f'Edge detection node started\n'
            f'  Algorithm: {self.algorithm}\n'
            f'  Canny thresholds: [{self.canny_t1}, {self.canny_t2}]'
        )

    def image_callback(self, msg):
        """Detect edges in incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply blur if requested
            if self.apply_blur:
                gray = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
            
            # Detect edges based on algorithm
            if self.algorithm == 'canny':
                edges = cv2.Canny(gray, self.canny_t1, self.canny_t2)
            elif self.algorithm == 'sobel':
                edges = self.sobel_edges(gray)
            elif self.algorithm == 'laplacian':
                edges = self.laplacian_edges(gray)
            elif self.algorithm == 'scharr':
                edges = self.scharr_edges(gray)
            else:
                edges = cv2.Canny(gray, self.canny_t1, self.canny_t2)
            
            # Publish edge image
            edges_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            edges_msg.header = msg.header
            self.edges_pub.publish(edges_msg)
            
            # Create overlay (edges on original image)
            overlay = cv_image.copy()
            overlay[edges > 0] = [0, 255, 0]  # Green edges
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)
            
            # Create visualization (side-by-side)
            edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            visualization = np.hstack([cv_image, edges_colored, overlay])
            vis_msg = self.bridge.cv2_to_imgmsg(visualization, encoding='bgr8')
            vis_msg.header = msg.header
            self.visualization_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error detecting edges: {e}')

    def sobel_edges(self, gray):
        """Apply Sobel edge detection."""
        grad_x = cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=self.sobel_ksize,
                          scale=self.sobel_scale, delta=self.sobel_delta)
        grad_y = cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=self.sobel_ksize,
                          scale=self.sobel_scale, delta=self.sobel_delta)
        
        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)
        
        edges = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        return edges

    def laplacian_edges(self, gray):
        """Apply Laplacian edge detection."""
        laplacian = cv2.Laplacian(gray, cv2.CV_16S, ksize=self.sobel_ksize)
        edges = cv2.convertScaleAbs(laplacian)
        return edges

    def scharr_edges(self, gray):
        """Apply Scharr edge detection."""
        grad_x = cv2.Scharr(gray, cv2.CV_16S, 1, 0)
        grad_y = cv2.Scharr(gray, cv2.CV_16S, 0, 1)
        
        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)
        
        edges = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        return edges


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
