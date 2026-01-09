#!/usr/bin/env python3
"""ArUco Detector Node - Detect ArUco markers for pose estimation and tracking."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.declare_parameter('dictionary_id', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.05)  # meters
        self.declare_parameter('draw_axes', True)
        
        dict_name = self.get_parameter('dictionary_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.draw_axes = self.get_parameter('draw_axes').value
        
        # Initialize ArUco dictionary
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        }
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dicts.get(dict_name, cv2.aruco.DICT_4X4_50)
        )
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseArray, 'aruco/poses', 10)
        self.vis_pub = self.create_publisher(Image, 'aruco/visualization', 10)
        
        self.get_logger().info(f'ArUco detector started (dict={dict_name})')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
            
            # Visualize
            vis_image = cv_image.copy()
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(vis_image, corners, ids)
                
                cv2.putText(vis_image, f'Detected: {len(ids)} markers', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Create pose array
                pose_array = PoseArray()
                pose_array.header = msg.header
                
                for i in range(len(ids)):
                    pose = Pose()
                    # In production, estimate pose using cv2.aruco.estimatePoseSingleMarkers
                    pose_array.poses.append(pose)
                
                self.pose_pub.publish(pose_array)
            else:
                cv2.putText(vis_image, 'No markers detected', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error detecting ArUco markers: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
