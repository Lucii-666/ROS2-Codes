#!/usr/bin/env python3
"""
Launch the full ROS pipeline - From Sensors to Actions
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    The complete ROS pipeline:
    Sensors → Topics → Nodes → Controllers → Actuators
    
    This is the machine before it learns to dream.
    """
    
    return LaunchDescription([
        
        # Sensors
        Node(
            package='pure_ros_fundamentals',
            executable='lidar_watchman',
            name='lidar_watchman',
            output='screen',
        ),
        
        Node(
            package='pure_ros_fundamentals',
            executable='encoder_monk',
            name='encoder_monk',
            output='screen',
        ),
        
        # Processing
        Node(
            package='pure_ros_fundamentals',
            executable='cartographer',
            name='cartographer',
            output='screen',
        ),
        
        # Control
        Node(
            package='pure_ros_fundamentals',
            executable='controller_soldier',
            name='controller_soldier',
            output='screen',
        ),
        
        # Monitoring
        Node(
            package='pure_ros_fundamentals',
            executable='sensor_orchestra',
            name='sensor_orchestra',
            output='screen',
        ),
        
        Node(
            package='pure_ros_fundamentals',
            executable='physics_monitor',
            name='physics_monitor',
            output='screen',
        ),
        
        # TF Tree
        Node(
            package='pure_ros_fundamentals',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen',
        ),
        
    ])
