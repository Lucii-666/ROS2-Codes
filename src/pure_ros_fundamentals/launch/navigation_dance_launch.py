#!/usr/bin/env python3
"""
Launch the Navigation Dance Demo
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Watch the 7-step navigation dance in action.
    Every ROS robot does this dance, 20-50 times per second.
    """
    
    return LaunchDescription([
        
        # The Dancer
        Node(
            package='pure_ros_fundamentals',
            executable='navigation_dance',
            name='navigation_dance',
            output='screen',
        ),
        
        # Support nodes
        Node(
            package='pure_ros_fundamentals',
            executable='encoder_monk',
            name='encoder_monk',
            output='screen',
        ),
        
        Node(
            package='pure_ros_fundamentals',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen',
        ),
        
    ])
