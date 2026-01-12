#!/usr/bin/env python3
"""
Physics Monitor Node
Watches Gazebo's lies - monitors mass, friction, collisions, gravity.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class PhysicsMonitorNode(Node):
    """
    Gazebo is a physics lie detector.
    This node monitors what Gazebo simulates:
    - Mass and inertia
    - Friction coefficients
    - Collision detection
    - Gravity effects
    
    ROS doesn't know it's fake. That's the point.
    """

    def __init__(self):
        super().__init__('physics_monitor')
        
        # Publisher
        self.status_pub = self.create_publisher(String, '/physics_status', 10)
        
        # Timer - report physics every 2 seconds
        self.timer = self.create_timer(2.0, self.report_physics)
        
        # Simulated physics parameters
        self.robot_mass = 10.0  # kg
        self.wheel_friction = 0.8
        self.gravity = -9.81  # m/s²
        self.collision_count = 0
        
        self.get_logger().info('⚗️ Physics Monitor active. Watching Gazebo...')

    def report_physics(self):
        """
        Report what the physics engine is simulating.
        In real world: these are physical constraints.
        In Gazebo: these are mathematical approximations.
        """
        status = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'physics': {
                'robot_mass_kg': self.robot_mass,
                'wheel_friction_coefficient': self.wheel_friction,
                'gravity_m_s2': self.gravity,
                'collision_count': self.collision_count,
                'physics_engine': 'ODE (Open Dynamics Engine)',
                'time_step_ms': 1.0,  # 1ms timestep
                'real_time_factor': 1.0,  # Runs at real-time speed
            },
            'constraints': {
                'max_wheel_torque_nm': 2.0,
                'max_velocity_m_s': 1.0,
                'wheel_slip': 'enabled',
                'air_resistance': 'disabled',
            },
            'truths': [
                'Gazebo simulates: mass, friction, collisions, gravity',
                'ROS does not know it is fake',
                'Your code works the same on real hardware',
                'That is why we use Gazebo - physics realism without hardware',
            ]
        }
        
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)
        
        self.get_logger().info(
            f'⚗️ Physics Report:\n'
            f'   Mass: {self.robot_mass}kg\n'
            f'   Friction: {self.wheel_friction}\n'
            f'   Gravity: {self.gravity}m/s²\n'
            f'   Real-time factor: 1.0 (running at real speed)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('⚗️ Physics Monitor shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
