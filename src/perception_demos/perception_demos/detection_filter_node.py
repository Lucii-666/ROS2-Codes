#!/usr/bin/env python3
"""Detection Filter Node - Filter detections by various criteria."""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DetectionFilterNode(Node):
    def __init__(self):
        super().__init__('detection_filter_node')
        
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('allowed_classes', [])
        self.declare_parameter('min_size', 50)
        self.declare_parameter('max_size', 1000)
        
        self.min_conf = self.get_parameter('min_confidence').value
        self.allowed_classes = self.get_parameter('allowed_classes').value
        self.min_size = self.get_parameter('min_size').value
        self.max_size = self.get_parameter('max_size').value
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, 'yolo/detections', self.detection_callback, 10
        )
        self.filtered_pub = self.create_publisher(Detection2DArray, 'detections/filtered', 10)
        
        self.get_logger().info(f'Detection filter started (min_conf={self.min_conf})')

    def detection_callback(self, msg):
        filtered_msg = Detection2DArray()
        filtered_msg.header = msg.header
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            class_name = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            size = max(detection.bbox.size_x, detection.bbox.size_y)
            
            # Apply filters
            if confidence < self.min_conf:
                continue
            if self.allowed_classes and class_name not in self.allowed_classes:
                continue
            if size < self.min_size or size > self.max_size:
                continue
            
            filtered_msg.detections.append(detection)
        
        self.filtered_pub.publish(filtered_msg)
        
        if len(filtered_msg.detections) != len(msg.detections):
            self.get_logger().info(
                f'Filtered: {len(msg.detections)} -> {len(filtered_msg.detections)}',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
