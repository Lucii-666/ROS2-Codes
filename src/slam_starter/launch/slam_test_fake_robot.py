import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Paths
    slam_starter_dir = get_package_share_directory('slam_starter')
    slam_config_file = os.path.join(slam_starter_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    # Fake scan publisher (simulates a robot with lidar)
    fake_scan_node = Node(
        package='slam_starter',
        executable='fake_scan_publisher',
        name='fake_scan_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # SLAM Monitor
    monitor_node = Node(
        package='slam_starter',
        executable='slam_monitor',
        name='slam_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(fake_scan_node)
    ld.add_action(slam_node)
    ld.add_action(monitor_node)
    
    return ld
