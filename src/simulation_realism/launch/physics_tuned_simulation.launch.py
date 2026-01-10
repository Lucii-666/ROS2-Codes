#!/usr/bin/env python3
"""
Launch file for physics-tuned simulation with realistic sensor noise.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_simulation_realism = get_package_share_directory('simulation_realism')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    world_file = os.path.join(pkg_simulation_realism, 'worlds', 'physics_tuned_world.sdf')
    config_file = os.path.join(pkg_simulation_realism, 'config', 'physics_params.yaml')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_with_sensors',
            '-file', os.path.join(pkg_simulation_realism, 'worlds', 'robot_with_sensors.sdf'),
            '-x', '0',
            '-y', '0',
            '-z', '0.15'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for sensor topics
    bridge_params = [
        {'topic': '/scan', 'ros_type': 'sensor_msgs/msg/LaserScan', 'gz_type': 'ignition.msgs.LaserScan'},
        {'topic': '/camera/image', 'ros_type': 'sensor_msgs/msg/Image', 'gz_type': 'ignition.msgs.Image'},
        {'topic': '/camera/depth', 'ros_type': 'sensor_msgs/msg/Image', 'gz_type': 'ignition.msgs.Image'},
        {'topic': '/imu', 'ros_type': 'sensor_msgs/msg/Imu', 'gz_type': 'ignition.msgs.IMU'},
        {'topic': '/odom', 'ros_type': 'nav_msgs/msg/Odometry', 'gz_type': 'ignition.msgs.Odometry'},
        {'topic': '/cmd_vel', 'ros_type': 'geometry_msgs/msg/Twist', 'gz_type': 'ignition.msgs.Twist'},
    ]
    
    bridge_nodes = []
    for bridge_param in bridge_params:
        bridge_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f"{bridge_param['topic']}@{bridge_param['ros_type']}@{bridge_param['gz_type']}"
                ],
                output='screen'
            )
        )
    
    # Sensor noise monitor node
    sensor_monitor = Node(
        package='simulation_realism',
        executable='sensor_noise_monitor',
        name='sensor_noise_monitor',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Physics statistics node
    physics_stats = Node(
        package='simulation_realism',
        executable='physics_stats_node',
        name='physics_stats_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz for visualization
    rviz_config = os.path.join(pkg_simulation_realism, 'config', 'simulation_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        spawn_robot,
        *bridge_nodes,
        sensor_monitor,
        physics_stats,
        rviz,
    ])
