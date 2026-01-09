#!/usr/bin/env python3
"""
Color Detection Node - Detect specific colors in images.

This node demonstrates:
- HSV color space conversion
- Color range thresholding
- Contour detection
- Color masking
- Multiple color detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, ColorRGBA
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectionNode(Node):
    """Detects specific colors in images."""

    def __init__(self):
        super().__init__('color_detection_node')
        
        # Declare parameters
        self.declare_parameter('target_color', 'red')  # red, green, blue, yellow, orange
        self.declare_parameter('hsv_lower', [0, 100, 100])
        self.declare_parameter('hsv_upper', [10, 255, 255])
        self.declare_parameter('min_area', 500)
        self.declare_parameter('detect_multiple_colors', False)
        self.declare_parameter('draw_contours', True)
        self.declare_parameter('draw_bounding_boxes', True)
        
        # Get parameters
        self.target_color = self.get_parameter('target_color').value
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        self.min_area = self.get_parameter('min_area').value
        self.detect_multiple = self.get_parameter('detect_multiple_colors').value
        self.draw_contours = self.get_parameter('draw_contours').value
        self.draw_boxes = self.get_parameter('draw_bounding_boxes').value
        
        # Define color ranges (HSV)
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'red2': ([170, 100, 100], [180, 255, 255]),  # Red wraps around
            'green': ([40, 40, 40], [80, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255]),
            'orange': ([10, 100, 100], [20, 255, 255]),
            'purple': ([130, 100, 100], [160, 255, 255]),
            'cyan': ([80, 100, 100], [100, 255, 255]),
        }
        
        # Use custom range if provided, otherwise use predefined
        if self.target_color in self.color_ranges:
            self.hsv_lower = np.array(self.color_ranges[self.target_color][0])
            self.hsv_upper = np.array(self.color_ranges[self.target_color][1])
        else:
            self.hsv_lower = np.array(hsv_lower)
            self.hsv_upper = np.array(hsv_upper)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.mask_pub = self.create_publisher(Image, 'color_detection/mask', 10)
        self.result_pub = self.create_publisher(Image, 'color_detection/result', 10)
        self.detection_pub = self.create_publisher(String, 'color_detection/info', 10)
        
        self.get_logger().info(
            f'Color detection node started\n'
            f'  Target color: {self.target_color}\n'
            f'  HSV range: {self.hsv_lower} - {self.hsv_upper}\n'
            f'  Min area: {self.min_area}'
        )

    def image_callback(self, msg):
        """Detect colors in incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
            
            # Handle red color (wraps around in HSV)
            if self.target_color == 'red':
                mask2 = cv2.inRange(hsv, 
                                   np.array(self.color_ranges['red2'][0]),
                                   np.array(self.color_ranges['red2'][1]))
                mask = cv2.bitwise_or(mask, mask2)
            
            # Apply morphological operations to clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create result image
            result = cv_image.copy()
            detection_count = 0
            total_area = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_area:
                    detection_count += 1
                    total_area += area
                    
                    # Draw contours
                    if self.draw_contours:
                        cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                    
                    # Draw bounding box
                    if self.draw_boxes:
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        
                        # Add label
                        label = f'{self.target_color} #{detection_count}'
                        cv2.putText(result, label, (x, y - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Draw center point
                    M = cv2.moments(contour)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
            
            # Add info text
            info_text = f'Detected: {detection_count} {self.target_color} objects'
            cv2.putText(result, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish mask
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
            
            # Publish result
            result_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            result_msg.header = msg.header
            self.result_pub.publish(result_msg)
            
            # Publish detection info
            detection_msg = String()
            detection_msg.data = f'{detection_count} {self.target_color} objects, total area: {total_area}'
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error detecting color: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
