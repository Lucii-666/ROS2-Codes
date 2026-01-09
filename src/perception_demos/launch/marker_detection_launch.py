"""Launch ArUco marker detection and pose estimation."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_demos',
            executable='usb_camera_node',
            name='camera',
            parameters=[{'camera_index': 0}]
        ),
        Node(
            package='perception_demos',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[{
                'dictionary_id': 'DICT_4X4_50',
                'marker_size': 0.05,
                'draw_axes': True,
            }],
            output='screen'
        ),
        Node(
            package='perception_demos',
            executable='marker_pose_estimator_node',
            name='pose_estimator',
            parameters=[{
                'marker_size': 0.05,
                'marker_type': 'aruco',
                'publish_tf': True,
            }]
        ),
    ])
