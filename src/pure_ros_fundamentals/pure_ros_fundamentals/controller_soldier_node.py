#!/usr/bin/env python3
"""
Controller Soldier Node
The Foot Soldier - follows orders, controls the wheels, fights friction.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math


class ControllerSoldierNode(Node):
    """
    The controller is a soldier following orders.
    "Go to X,Y" - he obeys.
    PID control. Pure math.
    """

    def __init__(self):
        super().__init__('controller_soldier')
        
        # Subscriber - receive orders (goal positions)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Publisher - send velocity commands to motors
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/controller_path', 10)
        
        # Timer - control loop runs at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # State
        self.current_pose = None
        self.goal_pose = None
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # PID parameters (tuned by hand, like ancient craftsmen)
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.tolerance = 0.1  # 10cm tolerance
        
        self.get_logger().info('⚔️ Soldier ready for orders. Awaiting commands...')

    def odom_callback(self, msg):
        """Track current position"""
        self.current_pose = msg.pose.pose
        
        # Record path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.path.poses.append(pose_stamped)
        
        # Keep path length reasonable
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

    def goal_callback(self, msg):
        """Receive new orders"""
        self.goal_pose = msg.pose
        self.get_logger().info(
            f'⚔️ New orders received! Moving to '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def control_loop(self):
        """
        The control loop - runs 20 times per second.
        This is the Swiss watch mechanism of robotics.
        """
        if self.current_pose is None or self.goal_pose is None:
            return
        
        # Calculate error
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if goal reached
        if distance < self.tolerance:
            # Stop
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info('⚔️ Goal reached! Standing at attention.')
            self.goal_pose = None
            return
        
        # Calculate desired angle
        desired_angle = math.atan2(dy, dx)
        
        # Get current angle from quaternion
        q = self.current_pose.orientation
        current_angle = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        # Angle error (normalized to [-pi, pi])
        angle_error = desired_angle - current_angle
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # PID control (proportional only for simplicity)
        cmd = Twist()
        
        # If angle error is large, turn in place
        if abs(angle_error) > 0.2:  # ~11 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = self.kp_angular * angle_error
        else:
            # Move forward and adjust angle
            cmd.linear.x = min(self.kp_linear * distance, 0.5)  # Max 0.5 m/s
            cmd.angular.z = self.kp_angular * angle_error
        
        self.cmd_vel_pub.publish(cmd)
        
        # Publish path for visualization
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        
        self.get_logger().debug(
            f'⚔️ Distance: {distance:.2f}m, Angle error: {math.degrees(angle_error):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSoldierNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('⚔️ Soldier stands down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
