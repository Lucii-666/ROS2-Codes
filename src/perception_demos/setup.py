from setuptools import setup
import os
from glob import glob

package_name = 'perception_demos'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Perception Developer',
    maintainer_email='dev@example.com',
    description='Comprehensive ROS2 perception examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Camera Drivers
            'usb_camera_node = perception_demos.usb_camera_node:main',
            'realsense_camera_node = perception_demos.realsense_camera_node:main',
            'zed_camera_node = perception_demos.zed_camera_node:main',
            'multi_camera_node = perception_demos.multi_camera_node:main',
            'camera_calibration_node = perception_demos.camera_calibration_node:main',
            
            # OpenCV Basics
            'image_subscriber = perception_demos.image_subscriber:main',
            'image_filter_node = perception_demos.image_filter_node:main',
            'edge_detection_node = perception_demos.edge_detection_node:main',
            'color_detection_node = perception_demos.color_detection_node:main',
            'image_blend_node = perception_demos.image_blend_node:main',
            'video_recorder_node = perception_demos.video_recorder_node:main',
            
            # Object Detection
            'yolo_detector_node = perception_demos.yolo_detector_node:main',
            'yolo_webcam_node = perception_demos.yolo_webcam_node:main',
            'object_tracker_node = perception_demos.object_tracker_node:main',
            'roi_extractor_node = perception_demos.roi_extractor_node:main',
            'detection_visualizer_node = perception_demos.detection_visualizer_node:main',
            'detection_filter_node = perception_demos.detection_filter_node:main',
            
            # Markers
            'apriltag_detector_node = perception_demos.apriltag_detector_node:main',
            'aruco_detector_node = perception_demos.aruco_detector_node:main',
            'marker_pose_estimator_node = perception_demos.marker_pose_estimator_node:main',
            'marker_generator_node = perception_demos.marker_generator_node:main',
            'multi_marker_tracker_node = perception_demos.multi_marker_tracker_node:main',
            
            # Depth & Point Clouds
            'depth_image_processor_node = perception_demos.depth_image_processor_node:main',
            'pointcloud_generator_node = perception_demos.pointcloud_generator_node:main',
            'pointcloud_filter_node = perception_demos.pointcloud_filter_node:main',
            'obstacle_detection_node = perception_demos.obstacle_detection_node:main',
            'object_3d_locator_node = perception_demos.object_3d_locator_node:main',
            
            # Advanced Perception
            'face_detection_node = perception_demos.face_detection_node:main',
            'pose_estimation_node = perception_demos.pose_estimation_node:main',
            'semantic_segmentation_node = perception_demos.semantic_segmentation_node:main',
        ],
    },
)
