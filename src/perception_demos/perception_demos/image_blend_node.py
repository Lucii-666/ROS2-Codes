#!/usr/bin/env python3
"""
Image Blend Node - Blend multiple image streams together.

This node demonstrates:
- Multi-image blending
- Alpha compositing
- Weighted overlay
- Screen blending
- Picture-in-picture
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageBlendNode(Node):
    """Blends multiple image streams."""

    def __init__(self):
        super().__init__('image_blend_node')
        
        # Declare parameters
        self.declare_parameter('blend_mode', 'weighted')  # weighted, overlay, screen, pip
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('beta', 0.5)
        self.declare_parameter('gamma', 0.0)
        self.declare_parameter('pip_scale', 0.25)
        self.declare_parameter('pip_position', 'bottom_right')  # top_left, top_right, bottom_left, bottom_right
        
        # Get parameters
        self.blend_mode = self.get_parameter('blend_mode').value
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.gamma = self.get_parameter('gamma').value
        self.pip_scale = self.get_parameter('pip_scale').value
        self.pip_position = self.get_parameter('pip_position').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Store latest images
        self.image1 = None
        self.image2 = None
        
        # Subscribers
        self.image1_sub = self.create_subscription(
            Image, 'image1', self.image1_callback, 10
        )
        self.image2_sub = self.create_subscription(
            Image, 'image2', self.image2_callback, 10
        )
        
        # Publisher
        self.blended_pub = self.create_publisher(Image, 'image_blended', 10)
        
        # Timer for blending
        self.timer = self.create_timer(0.033, self.blend_callback)  # ~30 Hz
        
        self.get_logger().info(
            f'Image blend node started\n'
            f'  Blend mode: {self.blend_mode}\n'
            f'  Alpha: {self.alpha}, Beta: {self.beta}'
        )

    def image1_callback(self, msg):
        """Store first image."""
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing image1: {e}')

    def image2_callback(self, msg):
        """Store second image."""
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing image2: {e}')

    def blend_callback(self):
        """Blend images based on mode."""
        if self.image1 is None or self.image2 is None:
            return
        
        try:
            # Ensure images are the same size for blending
            h1, w1 = self.image1.shape[:2]
            h2, w2 = self.image2.shape[:2]
            
            if self.blend_mode == 'pip':
                blended = self.picture_in_picture()
            else:
                # Resize to match if needed
                if (h1, w1) != (h2, w2):
                    self.image2 = cv2.resize(self.image2, (w1, h1))
                
                if self.blend_mode == 'weighted':
                    blended = self.weighted_blend()
                elif self.blend_mode == 'overlay':
                    blended = self.overlay_blend()
                elif self.blend_mode == 'screen':
                    blended = self.screen_blend()
                else:
                    blended = self.weighted_blend()
            
            # Publish blended image
            blended_msg = self.bridge.cv2_to_imgmsg(blended, encoding='bgr8')
            blended_msg.header.stamp = self.get_clock().now().to_msg()
            blended_msg.header.frame_id = 'blended_frame'
            self.blended_pub.publish(blended_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error blending images: {e}')

    def weighted_blend(self):
        """Perform weighted blending."""
        return cv2.addWeighted(self.image1, self.alpha, self.image2, self.beta, self.gamma)

    def overlay_blend(self):
        """Perform overlay blending."""
        # Normalize images to 0-1 range
        img1_norm = self.image1.astype(float) / 255.0
        img2_norm = self.image2.astype(float) / 255.0
        
        # Overlay formula
        mask = img1_norm < 0.5
        result = np.zeros_like(img1_norm)
        result[mask] = 2 * img1_norm[mask] * img2_norm[mask]
        result[~mask] = 1 - 2 * (1 - img1_norm[~mask]) * (1 - img2_norm[~mask])
        
        return (result * 255).astype(np.uint8)

    def screen_blend(self):
        """Perform screen blending."""
        # Normalize images
        img1_norm = self.image1.astype(float) / 255.0
        img2_norm = self.image2.astype(float) / 255.0
        
        # Screen formula: 1 - (1-a) * (1-b)
        result = 1 - (1 - img1_norm) * (1 - img2_norm)
        
        return (result * 255).astype(np.uint8)

    def picture_in_picture(self):
        """Create picture-in-picture effect."""
        result = self.image1.copy()
        
        # Resize second image
        h1, w1 = self.image1.shape[:2]
        small_h = int(h1 * self.pip_scale)
        small_w = int(w1 * self.pip_scale)
        small_img = cv2.resize(self.image2, (small_w, small_h))
        
        # Determine position
        if self.pip_position == 'top_left':
            y, x = 10, 10
        elif self.pip_position == 'top_right':
            y, x = 10, w1 - small_w - 10
        elif self.pip_position == 'bottom_left':
            y, x = h1 - small_h - 10, 10
        else:  # bottom_right
            y, x = h1 - small_h - 10, w1 - small_w - 10
        
        # Place small image
        result[y:y+small_h, x:x+small_w] = small_img
        
        # Draw border
        cv2.rectangle(result, (x-2, y-2), (x+small_w+2, y+small_h+2), (255, 255, 255), 2)
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ImageBlendNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
