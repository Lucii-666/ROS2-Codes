#!/usr/bin/env python3
"""
YOLO Detector Node - Object detection using YOLO (You Only Look Once).

This node demonstrates:
- YOLO model loading (YOLOv5, YOLOv8, etc.)
- Real-time object detection
- Bounding box visualization
- Confidence thresholding
- Class filtering
- FPS optimization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


class YOLODetectorNode(Node):
    """Performs object detection using YOLO."""

    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolov5s.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('detect_classes', [])  # Empty = all classes
        self.declare_parameter('use_gpu', False)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.input_size = self.get_parameter('input_size').value
        self.detect_classes = self.get_parameter('detect_classes').value
        use_gpu = self.get_parameter('use_gpu').value
        
        # COCO class names (80 classes)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Note: In production, load actual YOLO model:
        # import torch
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        # Or: from ultralytics import YOLO; self.model = YOLO('yolov8n.pt')
        
        self.get_logger().info(
            f'YOLO detector initialized (simulated mode)\n'
            f'  Model: {model_path}\n'
            f'  Confidence: {self.conf_threshold}\n'
            f'  Input size: {self.input_size}\n'
            f'  GPU: {use_gpu}'
        )
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(Detection2DArray, 'yolo/detections', 10)
        self.visualization_pub = self.create_publisher(Image, 'yolo/visualization', 10)

    def image_callback(self, msg):
        """Detect objects in incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # In production: run YOLO inference
            # results = self.model(cv_image)
            # detections = results.pandas().xyxy[0]
            
            # Simulated detections
            detections = self.simulate_detections(cv_image)
            
            # Create Detection2DArray message
            detection_array_msg = Detection2DArray()
            detection_array_msg.header = msg.header
            
            # Visualize
            vis_image = cv_image.copy()
            
            for det in detections:
                x1, y1, x2, y2, conf, class_id = det
                class_name = self.class_names[int(class_id)]
                
                # Filter by confidence and class
                if conf < self.conf_threshold:
                    continue
                if self.detect_classes and class_name not in self.detect_classes:
                    continue
                
                # Create Detection2D message
                detection_msg = Detection2D()
                detection_msg.bbox.center.position.x = (x1 + x2) / 2
                detection_msg.bbox.center.position.y = (y1 + y2) / 2
                detection_msg.bbox.size_x = x2 - x1
                detection_msg.bbox.size_y = y2 - y1
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = float(conf)
                detection_msg.results.append(hypothesis)
                
                detection_array_msg.detections.append(detection_msg)
                
                # Draw on visualization
                cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), 
                             (0, 255, 0), 2)
                label = f'{class_name}: {conf:.2f}'
                cv2.putText(vis_image, label, (int(x1), int(y1) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish detections
            self.detections_pub.publish(detection_array_msg)
            
            # Add detection count
            cv2.putText(vis_image, f'Detections: {len(detection_array_msg.detections)}',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.visualization_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in YOLO detection: {e}')

    def simulate_detections(self, image):
        """Simulate YOLO detections for demonstration."""
        h, w = image.shape[:2]
        detections = [
            [w*0.2, h*0.3, w*0.4, h*0.6, 0.92, 0],  # person
            [w*0.5, h*0.4, w*0.7, h*0.7, 0.85, 56],  # chair
            [w*0.1, h*0.1, w*0.3, h*0.3, 0.78, 62],  # laptop
        ]
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
