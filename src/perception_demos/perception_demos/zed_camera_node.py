#!/usr/bin/env python3
"""
ZED Camera Node - Interface with Stereolabs ZED stereo camera.

This node demonstrates:
- ZED SDK integration
- Stereo image publishing
- Depth map from stereo
- Spatial mapping
- Positional tracking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np


class ZEDCameraNode(Node):
    """Publishes stereo images and depth data from ZED camera."""

    def __init__(self):
        super().__init__('zed_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_model', 'ZED2')  # ZED, ZED2, ZED2i, ZEDM, ZEDX
        self.declare_parameter('resolution', 'HD720')  # HD2K, HD1080, HD720, VGA
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('depth_mode', 'PERFORMANCE')  # PERFORMANCE, QUALITY, ULTRA
        self.declare_parameter('enable_positional_tracking', False)
        self.declare_parameter('enable_spatial_mapping', False)
        
        # Get parameters
        model = self.get_parameter('camera_model').value
        resolution = self.get_parameter('resolution').value
        fps = self.get_parameter('frame_rate').value
        depth_mode = self.get_parameter('depth_mode').value
        enable_tracking = self.get_parameter('enable_positional_tracking').value
        enable_mapping = self.get_parameter('enable_spatial_mapping').value
        
        # Publishers
        self.left_pub = self.create_publisher(Image, 'zed/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'zed/right/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'zed/depth/depth_registered', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'zed/left/camera_info', 10)
        
        if enable_tracking:
            self.pose_pub = self.create_publisher(PoseStamped, 'zed/pose', 10)
        
        self.bridge = CvBridge()
        
        self.get_logger().info(
            f'ZED camera node initialized (simulated {model})\n'
            f'  Resolution: {resolution}\n'
            f'  Frame rate: {fps} fps\n'
            f'  Depth mode: {depth_mode}\n'
            f'  Tracking: {enable_tracking}, Mapping: {enable_mapping}'
        )
        
        # Note: Real implementation would use:
        # import pyzed.sl as sl
        # self.zed = sl.Camera()
        # init_params = sl.InitParameters()
        # init_params.camera_resolution = sl.RESOLUTION.HD720
        # init_params.camera_fps = fps
        # init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        # status = self.zed.open(init_params)
        
        # Timer for simulated data
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        """Publish ZED camera data (simulated)."""
        timestamp = self.get_clock().now().to_msg()
        
        # In real implementation:
        # if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
        #     self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
        #     self.zed.retrieve_image(self.right_image, sl.VIEW.RIGHT)
        #     self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
        
        # Simulated left image (720p)
        left_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        left_image[:360, :] = (50, 100, 150)  # Top half
        left_image[360:, :] = (150, 100, 50)  # Bottom half
        
        left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='bgr8')
        left_msg.header.stamp = timestamp
        left_msg.header.frame_id = 'zed_left_camera_frame'
        self.left_pub.publish(left_msg)
        
        # Simulated right image
        right_image = left_image.copy()
        right_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='bgr8')
        right_msg.header.stamp = timestamp
        right_msg.header.frame_id = 'zed_right_camera_frame'
        self.right_pub.publish(right_msg)
        
        # Simulated depth map (32-bit float, meters)
        depth_map = np.random.uniform(0.5, 10.0, (720, 1280)).astype(np.float32)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
        depth_msg.header.stamp = timestamp
        depth_msg.header.frame_id = 'zed_left_camera_frame'
        self.depth_pub.publish(depth_msg)
        
        # Camera info
        camera_info = CameraInfo()
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = 'zed_left_camera_frame'
        camera_info.width = 1280
        camera_info.height = 720
        self.left_info_pub.publish(camera_info)
        
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published {self.frame_count} ZED frames (simulated)')

    def destroy_node(self):
        """Clean up resources."""
        # Real implementation: self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
