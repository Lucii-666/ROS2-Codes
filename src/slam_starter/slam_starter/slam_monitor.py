#!/usr/bin/env python3
"""
Simple ROS2 SLAM Monitor Node
Subscribes to map and scan topics and provides basic status information
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import time


class SlamMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Status tracking
        self.map_count = 0
        self.scan_count = 0
        self.last_map_time = None
        self.last_scan_time = None
        self.map_size = None
        
        # Timer for status updates
        self.timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('SLAM Monitor started')
    
    def map_callback(self, msg):
        """Callback for map updates"""
        self.map_count += 1
        self.last_map_time = time.time()
        self.map_size = (msg.info.width, msg.info.height)
        
    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.scan_count += 1
        self.last_scan_time = time.time()
    
    def print_status(self):
        """Print current SLAM status"""
        current_time = time.time()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('SLAM Monitor Status:')
        self.get_logger().info(f'  Maps received: {self.map_count}')
        self.get_logger().info(f'  Scans received: {self.scan_count}')
        
        if self.map_size:
            self.get_logger().info(f'  Map size: {self.map_size[0]} x {self.map_size[1]}')
        
        if self.last_map_time:
            elapsed = current_time - self.last_map_time
            self.get_logger().info(f'  Last map: {elapsed:.1f}s ago')
        else:
            self.get_logger().warn('  No map received yet!')
            
        if self.last_scan_time:
            elapsed = current_time - self.last_scan_time
            self.get_logger().info(f'  Last scan: {elapsed:.1f}s ago')
        else:
            self.get_logger().warn('  No scan data received yet!')
            
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = SlamMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
