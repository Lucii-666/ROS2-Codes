#!/usr/bin/env python3
"""
LiDAR Watchman Node
The Watchman on the Tower - sees everything, reports distance to danger.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random


class LidarWatchmanNode(Node):
    """
    The LiDAR node is like a watchman on a medieval tower.
    It spins 360 degrees, measuring distance to every wall, every obstacle.
    No thinking. Just reporting facts.
    """

    def __init__(self):
        super().__init__('lidar_watchman')
        
        # Publisher - the pigeon carrying messages
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer - the clock that never stops
        self.timer = self.create_timer(0.1, self.scan_callback)  # 10 Hz
        
        # Configuration
        self.angle_min = -math.pi  # -180 degrees
        self.angle_max = math.pi   # +180 degrees
        self.angle_increment = math.pi / 180.0  # 1 degree steps
        self.range_min = 0.1  # 10 cm
        self.range_max = 10.0  # 10 meters
        
        self.get_logger().info('🗼 LiDAR Watchman on duty. Scanning the horizon...')

    def scan_callback(self):
        """
        Every 100ms, the watchman reports what he sees.
        This is deterministic measurement, not learning.
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        
        # Simulate a scan - in real world, this comes from hardware
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        msg.ranges = []
        msg.intensities = []
        
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            
            # Simulate walls at different distances
            # Front: wall at 3m
            # Sides: walls at 2m
            # Back: open space
            if -0.5 < angle < 0.5:  # Front cone
                distance = 3.0 + random.uniform(-0.05, 0.05)  # Sensor noise
            elif angle < -2.5 or angle > 2.5:  # Back cone
                distance = self.range_max  # No obstacle
            else:  # Sides
                distance = 2.0 + random.uniform(-0.05, 0.05)
            
            msg.ranges.append(distance)
            msg.intensities.append(100.0)  # Reflection strength
        
        self.publisher.publish(msg)
        
        # Report the closest obstacle
        closest = min(msg.ranges)
        if closest < self.range_max:
            self.get_logger().debug(f'Closest obstacle: {closest:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchmanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🗼 Watchman leaves his post.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
