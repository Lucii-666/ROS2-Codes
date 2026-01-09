#!/usr/bin/env python3
"""Obstacle Detection Node - Detect obstacles from depth/point cloud data."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import numpy as np


class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        
        self.declare_parameter('detection_range', 3.0)
        self.declare_parameter('min_obstacle_height', 0.1)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('use_clustering', True)
        
        self.detection_range = self.get_parameter('detection_range').value
        self.min_height = self.get_parameter('min_obstacle_height').value
        self.grid_res = self.get_parameter('grid_resolution').value
        self.use_clustering = self.get_parameter('use_clustering').value
        
        self.bridge = CvBridge()
        
        self.depth_sub = self.create_subscription(Image, 'camera/depth/image_raw',
                                                  self.depth_callback, 10)
        self.obstacles_pub = self.create_publisher(PoseArray, 'obstacles/poses', 10)
        self.vis_pub = self.create_publisher(Image, 'obstacles/visualization', 10)
        
        self.get_logger().info(f'Obstacle detection started (range={self.detection_range}m)')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Simple obstacle detection: find regions closer than threshold
            obstacle_mask = (depth_image > 0) & (depth_image < self.detection_range * 1000)
            
            # Visualization
            vis_image = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
            vis_image[obstacle_mask] = [0, 0, 255]  # Red for obstacles
            vis_image[~obstacle_mask] = [0, 255, 0]  # Green for free space
            
            # Find obstacle centers (simplified)
            obstacle_points = np.column_stack(np.where(obstacle_mask))
            
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            if len(obstacle_points) > 0:
                # Simple clustering by spatial proximity
                from sklearn.cluster import DBSCAN
                if self.use_clustering and len(obstacle_points) > 10:
                    clustering = DBSCAN(eps=20, min_samples=10).fit(obstacle_points)
                    unique_labels = set(clustering.labels_)
                    
                    for label in unique_labels:
                        if label == -1:  # Noise
                            continue
                        
                        cluster_points = obstacle_points[clustering.labels_ == label]
                        center = np.mean(cluster_points, axis=0)
                        
                        pose = Pose()
                        pose.position.x = float(center[1])  # Image x
                        pose.position.y = float(center[0])  # Image y
                        pose.position.z = float(np.mean(depth_image[cluster_points[:, 0],
                                                                    cluster_points[:, 1]]))
                        pose_array.poses.append(pose)
                        
                        # Draw cluster center
                        import cv2
                        cv2.circle(vis_image, (int(center[1]), int(center[0])),
                                 10, (255, 255, 0), -1)
            
            self.obstacles_pub.publish(pose_array)
            
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error detecting obstacles: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
