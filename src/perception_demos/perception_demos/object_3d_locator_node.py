#!/usr/bin/env python3
"""3D Object Locator Node - Locate objects in 3D space using depth data."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import numpy as np


class Object3DLocatorNode(Node):
    def __init__(self):
        super().__init__('object_3d_locator_node')
        
        self.declare_parameter('depth_sample_region', 'center')  # center, median, mean
        self.declare_parameter('depth_scale', 0.001)  # Depth units to meters
        
        self.depth_sample = self.get_parameter('depth_sample_region').value
        self.depth_scale = self.get_parameter('depth_scale').value
        
        self.bridge = CvBridge()
        self.latest_depth = None
        self.camera_matrix = None
        
        self.depth_sub = self.create_subscription(Image, 'camera/depth/image_raw',
                                                  self.depth_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, 'yolo/detections',
                                                      self.detection_callback, 10)
        
        self.poses_3d_pub = self.create_publisher(PoseArray, 'objects/poses_3d', 10)
        
        self.get_logger().info('3D object locator started')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detection_callback(self, msg):
        if self.latest_depth is None:
            return
        
        try:
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            for detection in msg.detections:
                if not detection.results:
                    continue
                
                # Get 2D bounding box
                cx = detection.bbox.center.position.x
                cy = detection.bbox.center.position.y
                w = detection.bbox.size_x
                h = detection.bbox.size_y
                
                x1, y1 = int(cx - w/2), int(cy - h/2)
                x2, y2 = int(cx + w/2), int(cy + h/2)
                
                # Sample depth in ROI
                roi_depth = self.latest_depth[y1:y2, x1:x2]
                
                if self.depth_sample == 'center':
                    depth = roi_depth[roi_depth.shape[0]//2, roi_depth.shape[1]//2]
                elif self.depth_sample == 'median':
                    depth = np.median(roi_depth[roi_depth > 0])
                else:  # mean
                    depth = np.mean(roi_depth[roi_depth > 0])
                
                if np.isnan(depth) or depth <= 0:
                    continue
                
                # Convert to meters
                z = depth * self.depth_scale
                
                # Project to 3D (simplified, assumes calibrated camera)
                # In production, use proper camera projection
                fx, fy = 525.0, 525.0  # Typical values
                cx_cam, cy_cam = 320.0, 240.0
                
                x = (cx - cx_cam) * z / fx
                y = (cy - cy_cam) * z / fy
                
                # Create 3D pose
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0
                
                pose_array.poses.append(pose)
            
            if pose_array.poses:
                self.poses_3d_pub.publish(pose_array)
                self.get_logger().info(f'Located {len(pose_array.poses)} objects in 3D',
                                      throttle_duration_sec=2.0)
            
        except Exception as e:
            self.get_logger().error(f'Error locating objects in 3D: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Object3DLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
