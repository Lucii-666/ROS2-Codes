"""Launch YOLO object detection pipeline."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_demos',
            executable='usb_camera_node',
            name='camera',
            parameters=[{'camera_index': 0, 'frame_rate': 30.0}]
        ),
        Node(
            package='perception_demos',
            executable='yolo_detector_node',
            name='yolo_detector',
            parameters=[{
                'confidence_threshold': 0.5,
                'nms_threshold': 0.4,
            }],
            output='screen'
        ),
        Node(
            package='perception_demos',
            executable='detection_visualizer_node',
            name='visualizer',
            parameters=[{
                'show_labels': True,
                'show_confidence': True,
            }]
        ),
    ])
