#!/usr/bin/env python3
"""
Simplified launch file for testing sensor noise without full Gazebo GUI.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulation_realism = get_package_share_directory('simulation_realism')
    config_file = os.path.join(pkg_simulation_realism, 'config', 'sensor_noise_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Sensor noise monitor
    sensor_monitor = Node(
        package='simulation_realism',
        executable='sensor_noise_monitor',
        name='sensor_noise_monitor',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        sensor_monitor,
    ])
