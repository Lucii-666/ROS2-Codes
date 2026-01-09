"""Launch camera nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_demos',
            executable='usb_camera_node',
            name='usb_camera',
            parameters=[{
                'camera_index': 0,
                'frame_rate': 30.0,
                'width': 640,
                'height': 480,
            }],
            output='screen'
        ),
    ])
