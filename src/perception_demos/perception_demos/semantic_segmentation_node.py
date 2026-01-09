#!/usr/bin/env python3
"""Semantic Segmentation Node - Perform pixel-wise semantic segmentation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__('semantic_segmentation_node')
        
        self.declare_parameter('model', 'deeplabv3')  # deeplabv3, fcn, segformer
        self.declare_parameter('backend', 'opencv')  # opencv, pytorch, onnx
        self.declare_parameter('input_size', 512)
        self.declare_parameter('apply_colormap', True)
        
        model = self.get_parameter('model').value
        backend = self.get_parameter('backend').value
        self.input_size = self.get_parameter('input_size').value
        self.apply_colormap = self.get_parameter('apply_colormap').value
        
        self.bridge = CvBridge()
        
        # Class names (PASCAL VOC / COCO)
        self.class_names = [
            'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus',
            'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike',
            'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'
        ]
        
        # Color map for classes
        self.colors = self.generate_color_map(len(self.class_names))
        
        # In production: load segmentation model
        # self.model = cv2.dnn.readNetFromTensorflow('frozen_inference_graph.pb')
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw',
                                                  self.image_callback, 10)
        self.segmentation_pub = self.create_publisher(Image, 'segmentation/mask', 10)
        self.overlay_pub = self.create_publisher(Image, 'segmentation/overlay', 10)
        
        self.get_logger().info(f'Semantic segmentation started (model={model}, backend={backend})')

    def generate_color_map(self, num_classes):
        """Generate distinct colors for each class."""
        np.random.seed(42)
        colors = np.random.randint(0, 255, (num_classes, 3), dtype=np.uint8)
        colors[0] = [0, 0, 0]  # Background is black
        return colors

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # In production: run segmentation model
            # blob = cv2.dnn.blobFromImage(cv_image, ...)
            # self.model.setInput(blob)
            # output = self.model.forward()
            
            # Simulated segmentation
            h, w = cv_image.shape[:2]
            segmentation_mask = np.zeros((h, w), dtype=np.uint8)
            
            # Simulate some segments
            segmentation_mask[h//4:3*h//4, w//4:3*w//4] = 15  # person
            segmentation_mask[0:h//4, :] = 0  # background
            segmentation_mask[h//3:2*h//3, w//3:2*w//3] = 9  # chair
            
            # Create colored segmentation
            if self.apply_colormap:
                segmentation_colored = self.colors[segmentation_mask]
            else:
                segmentation_colored = cv2.applyColorMap(
                    (segmentation_mask * 12).astype(np.uint8), cv2.COLORMAP_JET
                )
            
            # Create overlay
            overlay = cv2.addWeighted(cv_image, 0.6, segmentation_colored, 0.4, 0)
            
            # Add legend
            cv2.putText(overlay, 'Semantic Segmentation', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Count pixels per class
            unique, counts = np.unique(segmentation_mask, return_counts=True)
            y_offset = 60
            for class_id, count in zip(unique, counts):
                if class_id < len(self.class_names):
                    text = f'{self.class_names[class_id]}: {count/1000:.1f}k px'
                    cv2.putText(overlay, text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    y_offset += 20
            
            # Publish
            seg_msg = self.bridge.cv2_to_imgmsg(segmentation_colored, encoding='bgr8')
            seg_msg.header = msg.header
            self.segmentation_pub.publish(seg_msg)
            
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in semantic segmentation: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
