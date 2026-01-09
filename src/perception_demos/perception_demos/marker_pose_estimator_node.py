#!/usr/bin/env python3
"""Marker Pose Estimator Node - Estimate 6DOF pose from detected markers."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np


class MarkerPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('marker_pose_estimator_node')
        
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_type', 'aruco')  # aruco or apriltag
        self.declare_parameter('publish_tf', True)
        
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_type = self.get_parameter('marker_type').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, 'marker/pose', 10)
        
        self.get_logger().info(f'Marker pose estimator started ({self.marker_type})')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration received')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect markers and estimate pose
            # In production, use actual marker detection and pose estimation
            
            # Simulated pose
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = 0.5
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 1.0
            pose_msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(pose_msg)
            
            if self.publish_tf:
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = 'marker_frame'
                t.transform.translation.x = pose_msg.pose.position.x
                t.transform.translation.y = pose_msg.pose.position.y
                t.transform.translation.z = pose_msg.pose.position.z
                t.transform.rotation = pose_msg.pose.orientation
                self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error estimating marker pose: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
