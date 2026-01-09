"""Launch depth perception and point cloud generation."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_demos',
            executable='realsense_camera_node',
            name='realsense',
            parameters=[{
                'frame_rate': 30,
                'enable_depth': True,
                'enable_color': True,
                'enable_pointcloud': True,
            }],
            output='screen'
        ),
        Node(
            package='perception_demos',
            executable='depth_image_processor_node',
            name='depth_processor',
            parameters=[{
                'min_depth': 0.3,
                'max_depth': 5.0,
                'apply_colormap': True,
            }]
        ),
        Node(
            package='perception_demos',
            executable='pointcloud_generator_node',
            name='pointcloud_gen',
            parameters=[{
                'downsample_factor': 2,
                'max_depth': 5.0,
            }]
        ),
        Node(
            package='perception_demos',
            executable='obstacle_detection_node',
            name='obstacle_detector',
            parameters=[{
                'detection_range': 3.0,
                'min_obstacle_height': 0.1,
            }]
        ),
    ])
