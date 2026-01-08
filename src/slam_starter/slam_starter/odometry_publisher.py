#!/usr/bin/env python3
"""
ROS2 Odometry Publisher Node
Publishes odometry messages for robot localization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class OdometryPublisher(Node):
    """Node that publishes odometry data"""

    def __init__(self):
        super().__init__('odometry_publisher')
        
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.1  # m/s
        self.vtheta = 0.1  # rad/s
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Odometry Publisher Node started')
        self.get_logger().info('Publishing odometry to /odom at 20 Hz')

    def timer_callback(self):
        """Publish odometry and TF"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Update robot position (simple model)
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vtheta * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vtheta
        
        # Publish odometry
        self.publisher.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time
        
        # Log periodically
        if int(self.x * 10) % 10 == 0 and abs(delta_x) > 0.001:
            self.get_logger().info(
                f'Odom: x={self.x:.2f}m, y={self.y:.2f}m, θ={self.theta:.2f}rad'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
