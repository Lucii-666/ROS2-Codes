#!/usr/bin/env python3
"""ROI Extractor Node - Extract regions of interest from detections."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2


class ROIExtractorNode(Node):
    def __init__(self):
        super().__init__('roi_extractor_node')
        
        self.declare_parameter('padding', 10)
        self.declare_parameter('min_size', 50)
        self.declare_parameter('save_rois', False)
        
        self.padding = self.get_parameter('padding').value
        self.min_size = self.get_parameter('min_size').value
        self.save_rois = self.get_parameter('save_rois').value
        
        self.bridge = CvBridge()
        self.current_image = None
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, 'yolo/detections',
                                                      self.detection_callback, 10)
        
        self.roi_pubs = {}  # Dynamic publishers for each class
        
        self.get_logger().info('ROI extractor node started')

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_callback(self, msg):
        if self.current_image is None:
            return
        
        for i, detection in enumerate(msg.detections):
            if not detection.results:
                continue
            
            class_name = detection.results[0].hypothesis.class_id
            
            # Extract ROI coordinates
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y
            
            x1 = max(0, int(cx - w/2) - self.padding)
            y1 = max(0, int(cy - h/2) - self.padding)
            x2 = min(self.current_image.shape[1], int(cx + w/2) + self.padding)
            y2 = min(self.current_image.shape[0], int(cy + h/2) + self.padding)
            
            if (x2 - x1) < self.min_size or (y2 - y1) < self.min_size:
                continue
            
            # Extract ROI
            roi = self.current_image[y1:y2, x1:x2]
            
            # Create publisher for this class if not exists
            if class_name not in self.roi_pubs:
                self.roi_pubs[class_name] = self.create_publisher(
                    Image, f'roi/{class_name}', 10
                )
            
            # Publish ROI
            roi_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
            roi_msg.header.stamp = self.get_clock().now().to_msg()
            self.roi_pubs[class_name].publish(roi_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROIExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
