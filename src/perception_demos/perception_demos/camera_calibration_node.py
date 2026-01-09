#!/usr/bin/env python3
"""
Camera Calibration Node - Calibrate camera intrinsics and distortion.

This node demonstrates:
- Chessboard pattern detection
- Camera calibration using OpenCV
- Saving/loading calibration data
- Undistortion visualization
- Real-time calibration progress
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class CameraCalibrationNode(Node):
    """Performs camera calibration using chessboard patterns."""

    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # Declare parameters
        self.declare_parameter('chessboard_width', 9)
        self.declare_parameter('chessboard_height', 6)
        self.declare_parameter('square_size', 0.025)  # meters
        self.declare_parameter('num_images_required', 20)
        self.declare_parameter('calibration_file', 'camera_calibration.yaml')
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('capture_interval', 2.0)  # seconds
        
        # Get parameters
        self.board_width = self.get_parameter('chessboard_width').value
        self.board_height = self.get_parameter('chessboard_height').value
        self.square_size = self.get_parameter('square_size').value
        self.num_required = self.get_parameter('num_images_required').value
        self.calib_file = self.get_parameter('calibration_file').value
        auto_capture = self.get_parameter('auto_capture').value
        capture_interval = self.get_parameter('capture_interval').value
        
        # Calibration data
        self.board_size = (self.board_width, self.board_height)
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        
        # Prepare object points
        self.objp = np.zeros((self.board_width * self.board_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_width, 0:self.board_height].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Calibration results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.is_calibrated = False
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.visualization_pub = self.create_publisher(Image, 'calibration/visualization', 10)
        self.undistorted_pub = self.create_publisher(Image, 'calibration/undistorted', 10)
        self.status_pub = self.create_publisher(String, 'calibration/status', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'calibration/camera_info', 10)
        
        # Auto-capture timer
        if auto_capture:
            self.capture_timer = self.create_timer(capture_interval, self.auto_capture_callback)
            self.auto_capture_enabled = True
            self.last_capture_time = self.get_clock().now()
        else:
            self.auto_capture_enabled = False
        
        self.latest_frame = None
        
        self.get_logger().info(
            f'Camera calibration node started\n'
            f'  Chessboard: {self.board_width}x{self.board_height}\n'
            f'  Square size: {self.square_size}m\n'
            f'  Images required: {self.num_required}\n'
            f'  Auto-capture: {auto_capture}'
        )
        
        # Try to load existing calibration
        self.load_calibration()

    def image_callback(self, msg):
        """Process incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image.copy()
            
            # Find chessboard corners
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
            
            # Draw visualization
            vis_image = cv_image.copy()
            if ret:
                # Refine corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # Draw corners
                cv2.drawChessboardCorners(vis_image, self.board_size, corners_refined, ret)
                
                # Add text
                cv2.putText(vis_image, f'Pattern detected! ({len(self.obj_points)}/{self.num_required})',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(vis_image, f'Searching for pattern... ({len(self.obj_points)}/{self.num_required})',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.visualization_pub.publish(vis_msg)
            
            # Publish undistorted if calibrated
            if self.is_calibrated:
                undistorted = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
                undist_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
                undist_msg.header = msg.header
                self.undistorted_pub.publish(undist_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def auto_capture_callback(self):
        """Automatically capture calibration images."""
        if self.latest_frame is None or len(self.obj_points) >= self.num_required:
            return
        
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
        
        if ret:
            self.capture_calibration_image(gray, corners)

    def capture_calibration_image(self, gray, corners):
        """Capture a calibration image."""
        # Refine corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        self.obj_points.append(self.objp)
        self.img_points.append(corners_refined)
        
        self.get_logger().info(f'Captured calibration image {len(self.obj_points)}/{self.num_required}')
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Captured: {len(self.obj_points)}/{self.num_required}'
        self.status_pub.publish(status_msg)
        
        # Perform calibration if we have enough images
        if len(self.obj_points) >= self.num_required:
            self.perform_calibration(gray.shape[::-1])

    def perform_calibration(self, image_size):
        """Perform camera calibration."""
        self.get_logger().info('Performing camera calibration...')
        
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None
        )
        
        if ret:
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            self.is_calibrated = True
            
            # Calculate reprojection error
            mean_error = 0
            for i in range(len(self.obj_points)):
                img_points2, _ = cv2.projectPoints(
                    self.obj_points[i], rvecs[i], tvecs[i], mtx, dist
                )
                error = cv2.norm(self.img_points[i], img_points2, cv2.NORM_L2) / len(img_points2)
                mean_error += error
            mean_error /= len(self.obj_points)
            
            self.get_logger().info(f'Calibration successful! Mean error: {mean_error:.4f} pixels')
            
            # Save calibration
            self.save_calibration()
            
            # Publish camera info
            self.publish_camera_info()
        else:
            self.get_logger().error('Calibration failed!')

    def save_calibration(self):
        """Save calibration data to file."""
        calib_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'distortion_coefficients': self.dist_coeffs.tolist(),
            'image_width': int(self.latest_frame.shape[1]) if self.latest_frame is not None else 0,
            'image_height': int(self.latest_frame.shape[0]) if self.latest_frame is not None else 0,
        }
        
        try:
            with open(self.calib_file, 'w') as f:
                yaml.dump(calib_data, f)
            self.get_logger().info(f'Calibration saved to {self.calib_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')

    def load_calibration(self):
        """Load calibration data from file."""
        if not os.path.exists(self.calib_file):
            return
        
        try:
            with open(self.calib_file, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            self.camera_matrix = np.array(calib_data['camera_matrix'])
            self.dist_coeffs = np.array(calib_data['distortion_coefficients'])
            self.is_calibrated = True
            
            self.get_logger().info(f'Calibration loaded from {self.calib_file}')
            self.publish_camera_info()
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    def publish_camera_info(self):
        """Publish camera info message."""
        if not self.is_calibrated:
            return
        
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'camera_frame'
        
        if self.latest_frame is not None:
            camera_info.height = self.latest_frame.shape[0]
            camera_info.width = self.latest_frame.shape[1]
        
        camera_info.k = self.camera_matrix.flatten().tolist()
        camera_info.d = self.dist_coeffs.flatten().tolist()
        
        self.camera_info_pub.publish(camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
