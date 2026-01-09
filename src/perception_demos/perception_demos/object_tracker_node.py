#!/usr/bin/env python3
"""Object Tracker Node - Track detected objects across frames using various algorithms."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        
        self.declare_parameter('tracker_type', 'CSRT')  # KCF, CSRT, MedianFlow, MOSSE
        self.declare_parameter('max_age', 30)
        self.declare_parameter('min_hits', 3)
        
        self.tracker_type = self.get_parameter('tracker_type').value
        self.trackers = []
        self.track_ids = []
        self.next_id = 1
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, 'yolo/detections',
                                                      self.detection_callback, 10)
        
        self.tracked_pub = self.create_publisher(Image, 'tracker/visualization', 10)
        
        self.latest_detections = None
        self.get_logger().info(f'Object tracker started with {self.tracker_type}')

    def detection_callback(self, msg):
        self.latest_detections = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update existing trackers
            for i, (tracker, track_id) in enumerate(zip(self.trackers[:], self.track_ids[:])):
                success, box = tracker.update(cv_image)
                if success:
                    x, y, w, h = [int(v) for v in box]
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'ID: {track_id}', (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish tracked image
            tracked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            tracked_msg.header = msg.header
            self.tracked_pub.publish(tracked_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in tracking: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
