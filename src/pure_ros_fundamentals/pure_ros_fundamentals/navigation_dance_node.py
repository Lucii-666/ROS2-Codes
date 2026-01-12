#!/usr/bin/env python3
"""
Navigation Dance Node
The Navigation Dance - shows the 7-step waltz every ROS robot performs.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
import math
import time


class NavigationDanceNode(Node):
    """
    When you click "Send Goal" in RViz, this ancient dance begins:
    1. Nav2 calculates a path
    2. Global Planner draws the long road
    3. Local Planner dodges obstacles
    4. Controller sends velocity
    5. Encoders report back
    6. SLAM updates the map
    7. Loop 20-50 times per second
    
    This node demonstrates that dance in slow motion.
    """

    def __init__(self):
        super().__init__('navigation_dance')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/nav_dance_status', 10)
        
        # Subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        
        # State
        self.current_step = 0
        self.goal_pose = None
        
        self.get_logger().info('💃 Navigation Dancer ready. Send a goal to see the dance...')

    def goal_callback(self, msg):
        """When a goal arrives, start the dance"""
        self.goal_pose = msg
        self.current_step = 0
        
        self.get_logger().info('💃 Goal received! Beginning the sacred dance...')
        
        # Perform the dance
        self.perform_dance()

    def perform_dance(self):
        """
        The 7-step navigation dance.
        Each step explained. No magic. Pure determinism.
        """
        steps = [
            ("📍 Step 1: RECEIVE GOAL", self.step1_receive_goal),
            ("🗺️ Step 2: GLOBAL PLANNER - Draw the long road", self.step2_global_plan),
            ("🎯 Step 3: LOCAL PLANNER - Check for obstacles", self.step3_local_plan),
            ("⚙️ Step 4: CONTROLLER - Calculate velocities", self.step4_controller),
            ("🚗 Step 5: ACTUATORS - Move the wheels", self.step5_actuators),
            ("📿 Step 6: ENCODERS - Report position", self.step6_encoders),
            ("🗺️ Step 7: SLAM - Update the map", self.step7_slam),
        ]
        
        for i, (description, step_func) in enumerate(steps):
            self.current_step = i + 1
            
            # Announce step
            self.get_logger().info(f'\n{description}')
            msg = String()
            msg.data = f"Step {self.current_step}/7: {description}"
            self.status_pub.publish(msg)
            
            # Execute step
            step_func()
            
            # Pause for dramatic effect (in real robots, this is microseconds)
            time.sleep(1.0)
        
        self.get_logger().info('\n💃 Dance complete! This loop repeats 20-50 times per second.')

    def step1_receive_goal(self):
        """Step 1: The goal arrives from RViz or another node"""
        self.get_logger().info(
            f'   Goal coordinates: '
            f'({self.goal_pose.pose.position.x:.2f}, {self.goal_pose.pose.position.y:.2f})'
        )
        self.get_logger().info('   Robot acknowledges: "Roger that, moving out!"')

    def step2_global_plan(self):
        """Step 2: Plan a path from start to goal"""
        self.get_logger().info('   Calculating optimal path using A* algorithm...')
        self.get_logger().info('   A* invented in 1968 - older than email!')
        
        # Create a simple straight-line path for demonstration
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        # Create waypoints
        num_points = 10
        for i in range(num_points + 1):
            pose = PoseStamped()
            pose.header = path.header
            t = i / num_points
            pose.pose.position.x = t * self.goal_pose.pose.position.x
            pose.pose.position.y = t * self.goal_pose.pose.position.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.path_pub.publish(path)
        self.get_logger().info(f'   Path planned: {len(path.poses)} waypoints')

    def step3_local_plan(self):
        """Step 3: Check for nearby obstacles"""
        self.get_logger().info('   Scanning LiDAR data for obstacles...')
        self.get_logger().info('   Dynamic Window Approach (DWA) running...')
        self.get_logger().info('   Clearance check: ✓ Path is clear!')

    def step4_controller(self):
        """Step 4: Calculate wheel velocities"""
        self.get_logger().info('   PID Controller calculating velocities...')
        self.get_logger().info('   Kp=0.5, Ki=0.0, Kd=0.1 (tuned by hand)')
        
        # Simple velocity command
        cmd = Twist()
        cmd.linear.x = 0.2  # 0.2 m/s forward
        cmd.angular.z = 0.05  # Small turn
        
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info(
            f'   Command: Linear={cmd.linear.x:.2f}m/s, '
            f'Angular={cmd.angular.z:.2f}rad/s'
        )

    def step5_actuators(self):
        """Step 5: Send commands to motors"""
        self.get_logger().info('   Motors receiving commands...')
        self.get_logger().info('   Left wheel: PWM 150/255')
        self.get_logger().info('   Right wheel: PWM 145/255')
        self.get_logger().info('   Fighting friction, gravity, and inertia...')

    def step6_encoders(self):
        """Step 6: Encoders report back"""
        self.get_logger().info('   Encoders counting rotations...')
        self.get_logger().info('   Left wheel: 1247 ticks')
        self.get_logger().info('   Right wheel: 1251 ticks')
        self.get_logger().info('   Dead reckoning: Position updated')

    def step7_slam(self):
        """Step 7: Update the map"""
        self.get_logger().info('   SLAM algorithm running...')
        self.get_logger().info('   Particle filter with 500 particles')
        self.get_logger().info('   Map confidence: 87%')
        self.get_logger().info('   Loop closure: Not detected')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationDanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('💃 Dancer takes a bow.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
