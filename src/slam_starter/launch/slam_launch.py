import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get the directory of creating package
    slam_starter_dir = get_package_share_directory('slam_starter')
    
    # Define the config file to be used
    slam_config_file = os.path.join(slam_starter_dir, 'config', 'slam_toolbox_params.yaml')

    # Define the launch description
    ld = LaunchDescription()

    # Declare the launch argument
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    # Define the SLAM Toolbox node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          slam_config_file,
          {'use_sim_time': use_sim_time}
        ]
    )

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
