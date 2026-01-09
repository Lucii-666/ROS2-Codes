#!/usr/bin/env python3
"""Multi-Marker Tracker Node - Track multiple markers simultaneously."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np


class MultiMarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('multi_marker_tracker_node')
        
        self.declare_parameter('max_markers', 10)
        self.declare_parameter('marker_type', 'aruco')
        self.declare_parameter('track_history_length', 30)
        
        self.max_markers = self.get_parameter('max_markers').value
        self.marker_type = self.get_parameter('marker_type').value
        history_length = self.get_parameter('track_history_length').value
        
        self.bridge = CvBridge()
        self.marker_tracks = {}  # {marker_id: [pose_history]}
        self.history_length = history_length
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.poses_pub = self.create_publisher(PoseArray, 'markers/tracked_poses', 10)
        self.vis_pub = self.create_publisher(Image, 'markers/tracking_visualization', 10)
        
        self.get_logger().info(f'Multi-marker tracker started (max={self.max_markers})')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
            
            vis_image = cv_image.copy()
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(vis_image, corners, ids)
                
                for i, marker_id in enumerate(ids.flatten()):
                    # Calculate center
                    corner = corners[i][0]
                    center = np.mean(corner, axis=0)
                    
                    # Update track history
                    if marker_id not in self.marker_tracks:
                        self.marker_tracks[marker_id] = []
                    
                    self.marker_tracks[marker_id].append(center)
                    if len(self.marker_tracks[marker_id]) > self.history_length:
                        self.marker_tracks[marker_id].pop(0)
                    
                    # Draw track
                    track = np.array(self.marker_tracks[marker_id], dtype=np.int32)
                    if len(track) > 1:
                        cv2.polylines(vis_image, [track], False, (0, 255, 255), 2)
                    
                    # Draw marker ID and track length
                    cv2.putText(vis_image, f'ID:{marker_id} ({len(track)})',
                               (int(center[0]), int(center[1]) - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Create pose
                    pose = Pose()
                    pose.position.x = float(center[0])
                    pose.position.y = float(center[1])
                    pose_array.poses.append(pose)
                
                cv2.putText(vis_image, f'Tracking: {len(ids)} markers', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(vis_image, 'No markers detected', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish
            self.poses_pub.publish(pose_array)
            
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error tracking markers: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiMarkerTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
