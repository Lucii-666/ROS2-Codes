#!/usr/bin/env python3
"""
Physics Statistics Node
Monitors physics engine performance and contact dynamics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
from collections import deque
import time


class PhysicsStatsNode(Node):
    """
    Monitors physics simulation statistics including:
    - Real-time factor
    - Contact stability
    - Motion stability
    - Velocity tracking accuracy
    """
    
    def __init__(self):
        super().__init__('physics_stats_node')
        
        # Parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('window_size', 50)
        
        update_rate = self.get_parameter('update_rate').value
        self.window_size = self.get_parameter('window_size').value
        
        # Data storage
        self.odom_history = deque(maxlen=self.window_size)
        self.cmd_vel_history = deque(maxlen=self.window_size)
        self.last_odom_time = None
        self.sim_start_time = time.time()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.rtf_pub = self.create_publisher(Float64, '/physics_stats/rtf', 10)
        self.stability_pub = self.create_publisher(
            Float64MultiArray, '/physics_stats/stability', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / update_rate, self.publish_stats)
        
        self.get_logger().info('Physics Statistics Node Started')
    
    def odom_callback(self, msg):
        """Store odometry data."""
        current_time = self.get_clock().now()
        self.odom_history.append({
            'time': current_time,
            'linear_vel': msg.twist.twist.linear.x,
            'angular_vel': msg.twist.twist.angular.z,
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y
        })
        self.last_odom_time = current_time
    
    def cmd_vel_callback(self, msg):
        """Store commanded velocity."""
        self.cmd_vel_history.append({
            'time': self.get_clock().now(),
            'linear': msg.linear.x,
            'angular': msg.angular.z
        })
    
    def compute_rtf(self):
        """Compute real-time factor."""
        if len(self.odom_history) < 2:
            return 1.0
        
        wall_time_elapsed = time.time() - self.sim_start_time
        
        if self.last_odom_time is not None:
            sim_time = self.last_odom_time.nanoseconds / 1e9
            if wall_time_elapsed > 0:
                return sim_time / wall_time_elapsed
        
        return 1.0
    
    def compute_stability(self):
        """Compute motion stability metrics."""
        if len(self.odom_history) < 10:
            return {'jerk': 0.0, 'drift': 0.0}
        
        # Compute jerk (rate of acceleration change)
        velocities = [d['linear_vel'] for d in list(self.odom_history)]
        accelerations = np.diff(velocities)
        jerk = np.std(np.diff(accelerations)) if len(accelerations) > 1 else 0.0
        
        # Compute drift (deviation from commanded velocity)
        drift = 0.0
        if len(self.cmd_vel_history) > 0 and len(self.odom_history) > 0:
            cmd_linear = self.cmd_vel_history[-1]['linear']
            actual_linear = self.odom_history[-1]['linear_vel']
            drift = abs(cmd_linear - actual_linear)
        
        return {'jerk': float(jerk), 'drift': float(drift)}
    
    def publish_stats(self):
        """Publish physics statistics."""
        # Real-time factor
        rtf = self.compute_rtf()
        rtf_msg = Float64()
        rtf_msg.data = rtf
        self.rtf_pub.publish(rtf_msg)
        
        # Stability metrics
        stability = self.compute_stability()
        stability_msg = Float64MultiArray()
        stability_msg.data = [stability['jerk'], stability['drift']]
        self.stability_pub.publish(stability_msg)
        
        # Log periodically
        self.get_logger().info(
            f'Physics Stats - RTF: {rtf:.3f}, '
            f'Jerk: {stability["jerk"]:.4f}, '
            f'Vel Drift: {stability["drift"]:.3f} m/s',
            throttle_duration_sec=5.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsStatsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
