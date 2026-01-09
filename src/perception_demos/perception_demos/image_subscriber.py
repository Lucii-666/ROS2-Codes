#!/usr/bin/env python3
"""
Image Subscriber - Basic ROS2 image subscription with OpenCV visualization.

This node demonstrates:
- Subscribing to sensor_msgs/Image
- cv_bridge for image conversion
- OpenCV image display
- Image encoding handling
- FPS calculation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class ImageSubscriber(Node):
    """Subscribes to and displays images using OpenCV."""

    def __init__(self):
        super().__init__('image_subscriber')
        
        # Declare parameters
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('window_name', 'ROS2 Image Viewer')
        self.declare_parameter('show_fps', True)
        self.declare_parameter('resize_width', 0)  # 0 = no resize
        self.declare_parameter('resize_height', 0)
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.show_fps = self.get_parameter('show_fps').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # FPS calculation
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        self.get_logger().info(
            f'Image subscriber started\n'
            f'  Topic: {image_topic}\n'
            f'  Window: {self.window_name}'
        )
        
        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if requested
            if self.resize_width > 0 and self.resize_height > 0:
                cv_image = cv2.resize(cv_image, (self.resize_width, self.resize_height))
            
            # Calculate FPS
            self.frame_count += 1
            elapsed_time = time.time() - self.start_time
            if elapsed_time > 1.0:
                self.fps = self.frame_count / elapsed_time
                self.frame_count = 0
                self.start_time = time.time()
            
            # Add FPS overlay
            if self.show_fps:
                fps_text = f'FPS: {self.fps:.1f}'
                cv2.putText(cv_image, fps_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # Add timestamp
            timestamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            cv2.putText(cv_image, f'Time: {timestamp}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Display image
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        """Clean up resources."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
