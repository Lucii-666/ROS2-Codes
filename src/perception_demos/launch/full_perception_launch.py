"""Launch complete perception pipeline."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Camera
        Node(
            package='perception_demos',
            executable='usb_camera_node',
            name='camera',
        ),
        
        # Object Detection
        Node(
            package='perception_demos',
            executable='yolo_detector_node',
            name='yolo',
        ),
        
        # Marker Detection
        Node(
            package='perception_demos',
            executable='aruco_detector_node',
            name='aruco',
        ),
        
        # Face Detection
        Node(
            package='perception_demos',
            executable='face_detection_node',
            name='face_detector',
        ),
        
        # Color Detection
        Node(
            package='perception_demos',
            executable='color_detection_node',
            name='color_detector',
            parameters=[{'target_color': 'red'}]
        ),
        
        # Edge Detection
        Node(
            package='perception_demos',
            executable='edge_detection_node',
            name='edge_detector',
        ),
    ])
