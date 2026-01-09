#!/usr/bin/env python3
"""PointCloud Generator Node - Generate point clouds from depth + RGB images."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import struct


class PointCloudGeneratorNode(Node):
    def __init__(self):
        super().__init__('pointcloud_generator_node')
        
        self.declare_parameter('downsample_factor', 2)
        self.declare_parameter('max_depth', 5.0)
        
        self.downsample = self.get_parameter('downsample_factor').value
        self.max_depth = self.get_parameter('max_depth').value
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        self.depth_sub = self.create_subscription(Image, 'camera/depth/image_raw',
                                                  self.depth_callback, 10)
        self.color_sub = self.create_subscription(Image, 'camera/color/image_raw',
                                                  self.color_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, 'camera/camera_info',
                                                        self.camera_info_callback, 10)
        
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        self.latest_depth = None
        self.latest_color = None
        
        self.get_logger().info('Point cloud generator started')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.generate_pointcloud(msg.header)

    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def generate_pointcloud(self, header):
        if self.latest_depth is None or self.camera_matrix is None:
            return
        
        try:
            # Create simulated point cloud
            h, w = self.latest_depth.shape
            points = []
            
            # Downsample for performance
            for v in range(0, h, self.downsample):
                for u in range(0, w, self.downsample):
                    z = self.latest_depth[v, u] / 1000.0  # Convert to meters
                    
                    if z <= 0 or z > self.max_depth:
                        continue
                    
                    # Project to 3D
                    x = (u - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
                    y = (v - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]
                    
                    # Get color
                    if self.latest_color is not None:
                        b, g, r = self.latest_color[v, u]
                        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                    else:
                        rgb = 0xFFFFFF
                    
                    points.append([x, y, z, rgb])
            
            if not points:
                return
            
            # Create PointCloud2 message
            pc2_msg = self.create_pointcloud2(np.array(points), header)
            self.pointcloud_pub.publish(pc2_msg)
            
            self.get_logger().info(f'Published point cloud with {len(points)} points',
                                  throttle_duration_sec=5.0)
            
        except Exception as e:
            self.get_logger().error(f'Error generating point cloud: {e}')

    def create_pointcloud2(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 16
        pc2.row_step = pc2.point_step * len(points)
        pc2.is_dense = True
        pc2.data = points.tobytes()
        
        return pc2


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
