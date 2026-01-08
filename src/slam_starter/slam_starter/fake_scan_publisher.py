#!/usr/bin/env python3
"""
Fake laser scan publisher for testing SLAM without a real robot
Publishes simulated laser scan data in a circular pattern
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import numpy as np


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.3  # m/s
        self.angular_vel = 0.2  # rad/s
        
        # Scan parameters
        self.scan_angle_min = -math.pi
        self.scan_angle_max = math.pi
        self.scan_angle_increment = math.pi / 180.0  # 1 degree
        self.scan_time_increment = 0.0
        self.scan_range_min = 0.1
        self.scan_range_max = 10.0
        
        # Create a simple square room environment
        self.room_width = 10.0
        self.room_height = 10.0
        
        # Timers
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)  # 20 Hz
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Fake Scan Publisher started')
        self.get_logger().info(f'Simulating robot moving in a circle in a {self.room_width}x{self.room_height}m room')
    
    def simulate_room_scan(self):
        """Generate simulated laser scan for a square room"""
        num_readings = int((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_increment)
        ranges = []
        
        for i in range(num_readings):
            angle = self.scan_angle_min + i * self.scan_angle_increment
            
            # Transform angle to world frame
            world_angle = self.theta + angle
            
            # Calculate intersection with room walls
            # Check all 4 walls and find the closest intersection
            min_range = self.scan_range_max
            
            # Right wall (x = room_width/2)
            if math.cos(world_angle) > 0:
                dist = (self.room_width/2 - self.x) / math.cos(world_angle)
                if dist > 0:
                    min_range = min(min_range, dist)
            
            # Left wall (x = -room_width/2)
            if math.cos(world_angle) < 0:
                dist = (-self.room_width/2 - self.x) / math.cos(world_angle)
                if dist > 0:
                    min_range = min(min_range, dist)
            
            # Top wall (y = room_height/2)
            if math.sin(world_angle) > 0:
                dist = (self.room_height/2 - self.y) / math.sin(world_angle)
                if dist > 0:
                    min_range = min(min_range, dist)
            
            # Bottom wall (y = -room_height/2)
            if math.sin(world_angle) < 0:
                dist = (-self.room_height/2 - self.y) / math.sin(world_angle)
                if dist > 0:
                    min_range = min(min_range, dist)
            
            # Add some noise
            noise = np.random.normal(0, 0.01)
            measured_range = min_range + noise
            
            # Clamp to valid range
            measured_range = max(self.scan_range_min, min(self.scan_range_max, measured_range))
            ranges.append(measured_range)
        
        return ranges
    
    def publish_scan(self):
        """Publish laser scan message"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        
        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_increment
        scan.time_increment = self.scan_time_increment
        scan.scan_time = 0.1
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        
        scan.ranges = self.simulate_room_scan()
        
        self.scan_pub.publish(scan)
    
    def publish_odometry(self):
        """Publish odometry and TF"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update robot position (circular motion)
        self.theta += self.angular_vel * dt
        self.x = 2.0 * math.cos(self.theta)
        self.y = 2.0 * math.sin(self.theta)
        
        # Create quaternion from yaw
        quat = self.euler_to_quaternion(0, 0, self.theta)
        
        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish static TF: base_link -> base_scan
        t_scan = TransformStamped()
        t_scan.header.stamp = current_time.to_msg()
        t_scan.header.frame_id = 'base_link'
        t_scan.child_frame_id = 'base_scan'
        t_scan.transform.translation.x = 0.0
        t_scan.transform.translation.y = 0.0
        t_scan.transform.translation.z = 0.0
        t_scan.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t_scan)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
