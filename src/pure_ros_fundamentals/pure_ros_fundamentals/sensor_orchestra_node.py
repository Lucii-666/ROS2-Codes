#!/usr/bin/env python3
"""
Sensor Orchestra Node
The Conductor - synchronizes all sensors, creates harmony from chaos.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json


class SensorOrchestraNode(Node):
    """
    All sensors play their own tune.
    The orchestra conductor listens to everyone,
    ensures they're in sync, reports if anyone misses a beat.
    """

    def __init__(self):
        super().__init__('sensor_orchestra')
        
        # Subscribe to all sensors
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Publisher - report orchestra status
        self.status_pub = self.create_publisher(String, '/sensor_status', 10)
        
        # Timer - check health every second
        self.timer = self.create_timer(1.0, self.report_status)
        
        # Tracking
        self.last_scan_time = None
        self.last_camera_time = None
        self.last_joint_time = None
        self.last_odom_time = None
        
        self.scan_count = 0
        self.camera_count = 0
        self.joint_count = 0
        self.odom_count = 0
        
        self.get_logger().info('🎼 Orchestra conductor ready. Listening to all instruments...')

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        self.scan_count += 1

    def camera_callback(self, msg):
        self.last_camera_time = self.get_clock().now()
        self.camera_count += 1

    def joint_callback(self, msg):
        self.last_joint_time = self.get_clock().now()
        self.joint_count += 1

    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
        self.odom_count += 1

    def report_status(self):
        """
        Check if all sensors are playing.
        This is fault detection - no AI needed.
        """
        now = self.get_clock().now()
        timeout = 2.0  # 2 seconds
        
        status = {
            'timestamp': now.nanoseconds / 1e9,
            'sensors': {}
        }
        
        # Check LiDAR
        if self.last_scan_time:
            age = (now - self.last_scan_time).nanoseconds / 1e9
            status['sensors']['lidar'] = {
                'active': age < timeout,
                'age': age,
                'count': self.scan_count,
                'hz': self.scan_count / age if age > 0 else 0
            }
        else:
            status['sensors']['lidar'] = {'active': False}
        
        # Check camera
        if self.last_camera_time:
            age = (now - self.last_camera_time).nanoseconds / 1e9
            status['sensors']['camera'] = {
                'active': age < timeout,
                'age': age,
                'count': self.camera_count,
                'hz': self.camera_count / age if age > 0 else 0
            }
        else:
            status['sensors']['camera'] = {'active': False}
        
        # Check encoders
        if self.last_joint_time:
            age = (now - self.last_joint_time).nanoseconds / 1e9
            status['sensors']['encoders'] = {
                'active': age < timeout,
                'age': age,
                'count': self.joint_count,
                'hz': self.joint_count / age if age > 0 else 0
            }
        else:
            status['sensors']['encoders'] = {'active': False}
        
        # Check odometry
        if self.last_odom_time:
            age = (now - self.last_odom_time).nanoseconds / 1e9
            status['sensors']['odometry'] = {
                'active': age < timeout,
                'age': age,
                'count': self.odom_count,
                'hz': self.odom_count / age if age > 0 else 0
            }
        else:
            status['sensors']['odometry'] = {'active': False}
        
        # Publish status
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)
        
        # Log summary
        active_count = sum(1 for s in status['sensors'].values() if s.get('active', False))
        total_count = len(status['sensors'])
        
        self.get_logger().info(
            f'🎼 Orchestra status: {active_count}/{total_count} instruments playing'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorOrchestraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🎼 Concert concluded.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
