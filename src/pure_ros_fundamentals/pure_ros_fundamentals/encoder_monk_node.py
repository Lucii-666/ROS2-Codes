#!/usr/bin/env python3
"""
Encoder Monk Node
The Monk with Prayer Beads - counts every wheel rotation, never forgets.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class EncoderMonkNode(Node):
    """
    Encoders are monks counting prayer beads.
    Every wheel rotation is a bead.
    They never lose count, never get distracted.
    This is how robots know where they are.
    """

    def __init__(self):
        super().__init__('encoder_monk')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer - count every 50ms
        self.timer = self.create_timer(0.05, self.count_beads)
        
        # Robot state
        self.wheel_radius = 0.05  # 5 cm wheels
        self.wheel_base = 0.2  # 20 cm between wheels
        self.left_position = 0.0  # radians
        self.right_position = 0.0
        self.x = 0.0  # Robot position in world
        self.y = 0.0
        self.theta = 0.0  # Robot orientation
        
        # Simulated motion
        self.linear_vel = 0.2  # 0.2 m/s forward
        self.angular_vel = 0.1  # 0.1 rad/s turning
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('📿 Encoder Monk begins counting. Ohm...')

    def count_beads(self):
        """
        Every tick, count the beads (encoder ticks).
        Calculate where the robot has moved.
        This is dead reckoning - the ancient art of navigation.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update wheel positions based on velocity
        # This simulates what encoders would measure
        wheel_angular_vel = self.linear_vel / self.wheel_radius
        self.left_position += wheel_angular_vel * dt
        self.right_position += wheel_angular_vel * dt
        
        # Update robot pose (odometry)
        delta_s = self.linear_vel * dt  # Distance traveled
        delta_theta = self.angular_vel * dt  # Angle changed
        
        # Update pose
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)
        self.theta += delta_theta
        
        # Publish joint states (wheel positions)
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [self.left_position, self.right_position]
        joint_msg.velocity = [wheel_angular_vel, wheel_angular_vel]
        self.joint_pub.publish(joint_msg)
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        odom_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.theta)
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom_msg)
        
        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.theta)
        
        self.tf_broadcaster.sendTransform(t)
        
        if int(self.left_position) % 10 == 0:
            self.get_logger().debug(
                f'📿 Position: ({self.x:.2f}, {self.y:.2f}), '
                f'Beads counted: {int(self.left_position)}'
            )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q


def main(args=None):
    rclpy.init(args=args)
    node = EncoderMonkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('📿 Monk stops counting.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
