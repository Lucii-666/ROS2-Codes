#!/usr/bin/env python3
"""
Cartographer Node
The Cartographer - draws maps from chaos, finds patterns in sensor noise.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
import numpy as np
import math


class CartographerNode(Node):
    """
    The cartographer listens to the watchman (LiDAR) and the monk (encoders).
    From their whispers, he draws a map of the world.
    Pure geometry and probability.
    """

    def __init__(self):
        super().__init__('cartographer')
        
        # Subscribers - listening to multiple sources
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Publisher - share the map with everyone
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer - update map periodically
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Map parameters
        self.resolution = 0.05  # 5cm per cell
        self.width = 200  # 10 meters
        self.height = 200
        self.map_data = np.zeros((self.height, self.width), dtype=np.int8)
        self.map_data.fill(-1)  # Unknown initially
        
        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.get_logger().info('🗺️ Cartographer ready. Drawing the world...')

    def odom_callback(self, msg):
        """Remember where the robot is"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.robot_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def scan_callback(self, msg):
        """
        When the watchman reports, the cartographer updates his map.
        Ray tracing - ancient geometry from the Greeks.
        """
        # Convert robot position to grid coordinates
        origin_x = self.width // 2
        origin_y = self.height // 2
        
        robot_grid_x = int(origin_x + self.robot_x / self.resolution)
        robot_grid_y = int(origin_y + self.robot_y / self.resolution)
        
        # Process each laser ray
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Calculate endpoint of laser ray
                world_angle = self.robot_theta + angle
                end_x = self.robot_x + r * math.cos(world_angle)
                end_y = self.robot_y + r * math.sin(world_angle)
                
                end_grid_x = int(origin_x + end_x / self.resolution)
                end_grid_y = int(origin_y + end_y / self.resolution)
                
                # Ray tracing - mark cells as free or occupied
                self.bresenham_line(
                    robot_grid_x, robot_grid_y,
                    end_grid_x, end_grid_y
                )
            
            angle += msg.angle_increment

    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm - invented in 1962.
        Older than most programming languages.
        This is what "deterministic" means.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # Mark cells along the ray as free
            if 0 <= x < self.width and 0 <= y < self.height:
                if self.map_data[y, x] == -1:  # Unknown
                    self.map_data[y, x] = 0  # Free
            
            if x == x1 and y == y1:
                # Mark endpoint as occupied
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.map_data[y, x] = 100  # Occupied
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_map(self):
        """Share the map with the world"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Map origin (bottom-left corner)
        msg.info.origin.position.x = -self.width * self.resolution / 2
        msg.info.origin.position.y = -self.height * self.resolution / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten map data
        msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(msg)
        self.get_logger().debug('🗺️ Map updated')


def main(args=None):
    rclpy.init(args=args)
    node = CartographerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🗺️ Cartographer puts down his pen.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
