#!/usr/bin/env python3
"""
Launch all sensor nodes - The Orchestra Assembles
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch all sensor nodes together.
    Watch them communicate. No AI. Pure determinism.
    """
    
    return LaunchDescription([
        
        # The Watchman on the Tower
        Node(
            package='pure_ros_fundamentals',
            executable='lidar_watchman',
            name='lidar_watchman',
            output='screen',
            parameters=[],
        ),
        
        # The Camera that Gossips
        Node(
            package='pure_ros_fundamentals',
            executable='camera_gossip',
            name='camera_gossip',
            output='screen',
            parameters=[],
        ),
        
        # The Monk with Prayer Beads
        Node(
            package='pure_ros_fundamentals',
            executable='encoder_monk',
            name='encoder_monk',
            output='screen',
            parameters=[],
        ),
        
        # The Orchestra Conductor
        Node(
            package='pure_ros_fundamentals',
            executable='sensor_orchestra',
            name='sensor_orchestra',
            output='screen',
            parameters=[],
        ),
        
        # The TF Tree Broadcaster
        Node(
            package='pure_ros_fundamentals',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen',
            parameters=[],
        ),
        
    ])
