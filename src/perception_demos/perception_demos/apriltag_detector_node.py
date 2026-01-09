#!/usr/bin/env python3
"""AprilTag Detector Node - Detect and decode AprilTag fiducial markers."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        
        self.declare_parameter('tag_family', 'tag36h11')  # tag36h11, tag25h9, tag16h5
        self.declare_parameter('tag_size', 0.16)  # meters
        self.declare_parameter('draw_detections', True)
        
        self.tag_family = self.get_parameter('tag_family').value
        self.tag_size = self.get_parameter('tag_size').value
        self.draw_detections = self.get_parameter('draw_detections').value
        
        self.bridge = CvBridge()
        
        # Note: In production, initialize AprilTag detector:
        # import apriltag
        # self.detector = apriltag.Detector(apriltag.DetectorOptions(families=self.tag_family))
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseArray, 'apriltag/poses', 10)
        self.vis_pub = self.create_publisher(Image, 'apriltag/visualization', 10)
        
        self.get_logger().info(f'AprilTag detector started (family={self.tag_family})')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # In production: detect tags
            # results = self.detector.detect(gray)
            
            # Simulated detection
            vis_image = cv_image.copy()
            cv2.putText(vis_image, f'AprilTag {self.tag_family} Detector', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(vis_image, 'Searching for tags...', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Simulate a tag detection
            h, w = cv_image.shape[:2]
            corners = np.array([[w*0.3, h*0.3], [w*0.5, h*0.3], 
                               [w*0.5, h*0.5], [w*0.3, h*0.5]], dtype=np.int32)
            cv2.polylines(vis_image, [corners], True, (0, 255, 0), 2)
            cv2.putText(vis_image, 'ID: 42', (int(w*0.3), int(h*0.3)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error detecting AprilTags: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
