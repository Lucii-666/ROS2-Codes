#!/usr/bin/env python3
"""
ROS2 Multi-Node Launch File
Launches multiple nodes together
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes"""
    
    return LaunchDescription([
        # Publisher node
        Node(
            package='slam_starter',
            executable='simple_publisher',
            name='publisher_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Subscriber node
        Node(
            package='slam_starter',
            executable='simple_subscriber',
            name='subscriber_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # TF Broadcaster
        Node(
            package='slam_starter',
            executable='tf_broadcaster',
            name='tf_broadcaster_node',
            output='screen'
        ),
        
        # TF Listener
        Node(
            package='slam_starter',
            executable='tf_listener',
            name='tf_listener_node',
            output='screen'
        ),
    ])
