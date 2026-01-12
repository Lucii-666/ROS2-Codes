from setuptools import setup
import os
from glob import glob

package_name = 'pure_ros_fundamentals'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS Samurai',
    maintainer_email='robot@example.com',
    description='The bare-metal truth of ROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_watchman = pure_ros_fundamentals.lidar_watchman_node:main',
            'camera_gossip = pure_ros_fundamentals.camera_gossip_node:main',
            'encoder_monk = pure_ros_fundamentals.encoder_monk_node:main',
            'cartographer = pure_ros_fundamentals.cartographer_node:main',
            'controller_soldier = pure_ros_fundamentals.controller_soldier_node:main',
            'sensor_orchestra = pure_ros_fundamentals.sensor_orchestra_node:main',
            'navigation_dance = pure_ros_fundamentals.navigation_dance_node:main',
            'tf_broadcaster = pure_ros_fundamentals.tf_broadcaster_node:main',
            'physics_monitor = pure_ros_fundamentals.physics_monitor_node:main',
        ],
    },
)
