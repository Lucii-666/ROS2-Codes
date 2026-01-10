#!/usr/bin/env python3
"""
Sensor Noise Monitor Node
Monitors sensor data and provides statistics on noise characteristics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque


class SensorNoiseMonitor(Node):
    """
    Monitors sensor data streams and computes noise statistics.
    Useful for validating sensor noise models and tuning parameters.
    """
    
    def __init__(self):
        super().__init__('sensor_noise_monitor')
        
        # Declare parameters
        self.declare_parameter('window_size', 100)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('enable_lidar_stats', True)
        self.declare_parameter('enable_imu_stats', True)
        self.declare_parameter('enable_odom_stats', True)
        
        # Get parameters
        self.window_size = self.get_parameter('window_size').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Data storage for statistics
        self.lidar_ranges = deque(maxlen=self.window_size)
        self.imu_angular_vel = {'x': deque(maxlen=self.window_size),
                                 'y': deque(maxlen=self.window_size),
                                 'z': deque(maxlen=self.window_size)}
        self.imu_linear_acc = {'x': deque(maxlen=self.window_size),
                                'y': deque(maxlen=self.window_size),
                                'z': deque(maxlen=self.window_size)}
        self.odom_velocities = {'linear': deque(maxlen=self.window_size),
                                'angular': deque(maxlen=self.window_size)}
        
        # Subscribers
        if self.get_parameter('enable_lidar_stats').value:
            self.lidar_sub = self.create_subscription(
                LaserScan, '/scan', self.lidar_callback, 10)
            
        if self.get_parameter('enable_imu_stats').value:
            self.imu_sub = self.create_subscription(
                Imu, '/imu', self.imu_callback, 10)
        
        if self.get_parameter('enable_odom_stats').value:
            self.odom_sub = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers for statistics
        self.lidar_stats_pub = self.create_publisher(
            Float64MultiArray, '/sensor_stats/lidar', 10)
        self.imu_stats_pub = self.create_publisher(
            Float64MultiArray, '/sensor_stats/imu', 10)
        
        # Timer for publishing statistics
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_statistics)
        
        self.get_logger().info('Sensor Noise Monitor Node Started')
        self.get_logger().info(f'Window size: {self.window_size}')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
    
    def lidar_callback(self, msg):
        """Process LiDAR data and store for statistics."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.lidar_ranges.append(valid_ranges)
    
    def imu_callback(self, msg):
        """Process IMU data and store for statistics."""
        self.imu_angular_vel['x'].append(msg.angular_velocity.x)
        self.imu_angular_vel['y'].append(msg.angular_velocity.y)
        self.imu_angular_vel['z'].append(msg.angular_velocity.z)
        
        self.imu_linear_acc['x'].append(msg.linear_acceleration.x)
        self.imu_linear_acc['y'].append(msg.linear_acceleration.y)
        self.imu_linear_acc['z'].append(msg.linear_acceleration.z)
    
    def odom_callback(self, msg):
        """Process odometry data and store for statistics."""
        linear_vel = np.sqrt(msg.twist.twist.linear.x**2 + 
                            msg.twist.twist.linear.y**2)
        self.odom_velocities['linear'].append(linear_vel)
        self.odom_velocities['angular'].append(msg.twist.twist.angular.z)
    
    def compute_statistics(self, data):
        """Compute mean, std, min, max from data."""
        if len(data) == 0:
            return {'mean': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0}
        
        arr = np.array(data)
        return {
            'mean': float(np.mean(arr)),
            'std': float(np.std(arr)),
            'min': float(np.min(arr)),
            'max': float(np.max(arr))
        }
    
    def publish_statistics(self):
        """Publish sensor noise statistics."""
        # LiDAR statistics
        if len(self.lidar_ranges) > 10:
            all_ranges = [r for scan in self.lidar_ranges for r in scan]
            stats = self.compute_statistics(all_ranges)
            
            msg = Float64MultiArray()
            msg.data = [stats['mean'], stats['std'], stats['min'], stats['max']]
            self.lidar_stats_pub.publish(msg)
            
            self.get_logger().info(
                f'LiDAR Stats - Mean: {stats["mean"]:.3f}m, '
                f'Std: {stats["std"]:.3f}m, '
                f'Range: [{stats["min"]:.3f}, {stats["max"]:.3f}]m',
                throttle_duration_sec=5.0
            )
        
        # IMU statistics
        if len(self.imu_angular_vel['x']) > 10:
            gyro_x_stats = self.compute_statistics(self.imu_angular_vel['x'])
            gyro_z_stats = self.compute_statistics(self.imu_angular_vel['z'])
            acc_x_stats = self.compute_statistics(self.imu_linear_acc['x'])
            acc_z_stats = self.compute_statistics(self.imu_linear_acc['z'])
            
            self.get_logger().info(
                f'IMU Stats - Gyro Z std: {gyro_z_stats["std"]:.4f} rad/s, '
                f'Acc X std: {acc_x_stats["std"]:.3f} m/s², '
                f'Acc Z mean: {acc_z_stats["mean"]:.3f} m/s² (gravity)',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorNoiseMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
