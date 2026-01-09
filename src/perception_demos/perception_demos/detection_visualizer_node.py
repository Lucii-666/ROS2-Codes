#!/usr/bin/env python3
"""Detection Visualizer Node - Advanced visualization of object detections."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectionVisualizerNode(Node):
    def __init__(self):
        super().__init__('detection_visualizer_node')
        
        self.declare_parameter('show_labels', True)
        self.declare_parameter('show_confidence', True)
        self.declare_parameter('show_count', True)
        self.declare_parameter('box_thickness', 2)
        
        self.show_labels = self.get_parameter('show_labels').value
        self.show_conf = self.get_parameter('show_confidence').value
        self.show_count = self.get_parameter('show_count').value
        self.box_thickness = self.get_parameter('box_thickness').value
        
        self.bridge = CvBridge()
        self.current_image = None
        self.class_colors = {}
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, 'yolo/detections',
                                                      self.detection_callback, 10)
        
        self.vis_pub = self.create_publisher(Image, 'detections/advanced_visualization', 10)
        
        self.get_logger().info('Detection visualizer started')

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_class_color(self, class_name):
        if class_name not in self.class_colors:
            self.class_colors[class_name] = tuple(np.random.randint(0, 255, 3).tolist())
        return self.class_colors[class_name]

    def detection_callback(self, msg):
        if self.current_image is None:
            return
        
        vis_image = self.current_image.copy()
        class_counts = {}
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            class_name = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            
            # Count detections by class
            class_counts[class_name] = class_counts.get(class_name, 0) + 1
            
            # Get bounding box
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y
            
            x1, y1 = int(cx - w/2), int(cy - h/2)
            x2, y2 = int(cx + w/2), int(cy + h/2)
            
            # Get color for this class
            color = self.get_class_color(class_name)
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, self.box_thickness)
            
            # Draw label
            if self.show_labels or self.show_conf:
                label = class_name if self.show_labels else ''
                if self.show_conf:
                    label += f' {confidence:.2f}' if label else f'{confidence:.2f}'
                
                # Label background
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(vis_image, (x1, y1 - label_h - 10), (x1 + label_w, y1), color, -1)
                cv2.putText(vis_image, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw statistics
        if self.show_count:
            y_offset = 30
            for class_name, count in class_counts.items():
                text = f'{class_name}: {count}'
                cv2.putText(vis_image, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_offset += 25
        
        # Publish visualization
        vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
