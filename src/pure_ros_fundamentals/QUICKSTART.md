# Quick Start Guide

## Installation

```bash
cd ~/ROS  # Your workspace
colcon build --packages-select pure_ros_fundamentals
source install/setup.bash
```

## Basic Demos

### 1. Sensor Orchestra (Recommended First)

Watch all sensors report in real-time:

```bash
ros2 launch pure_ros_fundamentals sensors_launch.py
```

In another terminal:
```bash
ros2 topic echo /sensor_status
```

### 2. Navigation Dance (Most Educational)

See the 7-step navigation process:

```bash
ros2 launch pure_ros_fundamentals navigation_dance_launch.py
```

Send a goal:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{pose: {position: {x: 2.0, y: 1.0}}}"
```

Watch the terminal for step-by-step explanation!

### 3. Full Pipeline (Complete System)

```bash
ros2 launch pure_ros_fundamentals full_pipeline_launch.py
```

## Visualization

### RViz Setup

```bash
rviz2
```

Add these:
- **LaserScan**: `/scan`
- **Map**: `/map`
- **Path**: `/controller_path`
- **TF**: (enable all frames)
- **Image**: `/camera/image_raw`

### Check Topics

```bash
# See all active topics
ros2 topic list

# Monitor sensor data
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /joint_states

# Check system status
ros2 topic echo /sensor_status
ros2 topic echo /physics_status
```

## Individual Nodes

Run standalone to understand each piece:

```bash
# Sensors
ros2 run pure_ros_fundamentals lidar_watchman
ros2 run pure_ros_fundamentals camera_gossip
ros2 run pure_ros_fundamentals encoder_monk

# Processing
ros2 run pure_ros_fundamentals cartographer
ros2 run pure_ros_fundamentals controller_soldier

# Monitoring
ros2 run pure_ros_fundamentals sensor_orchestra
ros2 run pure_ros_fundamentals physics_monitor

# Infrastructure
ros2 run pure_ros_fundamentals tf_broadcaster
```

## Understanding the Output

Each node uses emojis to identify itself:

- 🗼 **LiDAR Watchman** - Scanning the horizon
- 📷 **Camera Gossip** - Capturing images
- 📿 **Encoder Monk** - Counting wheel rotations
- 🗺️ **Cartographer** - Drawing the map
- ⚔️ **Controller Soldier** - Following orders
- 🎼 **Sensor Orchestra** - Monitoring all sensors
- 💃 **Navigation Dance** - Showing the process
- ⚗️ **Physics Monitor** - Watching simulation
- 🌳 **TF Broadcaster** - Managing coordinate frames

## Common Commands

### Send a Goal to the Controller

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing the tree
```

### Monitor Node Status

```bash
ros2 node list
ros2 node info /lidar_watchman
```

## Troubleshooting

### Nodes not appearing?

```bash
# Make sure you built and sourced
colcon build --packages-select pure_ros_fundamentals
source install/setup.bash
```

### No topics appearing?

```bash
# Check if nodes are running
ros2 node list

# Check node logs
ros2 run pure_ros_fundamentals lidar_watchman
```

### Want more detail?

```bash
# Change log level to DEBUG
ros2 run pure_ros_fundamentals lidar_watchman --ros-args --log-level debug
```

## Next Steps

1. ✅ Run each demo
2. ✅ Read the node code (it's heavily commented)
3. ✅ Modify parameters in `config/ros_params.yaml`
4. ✅ Create your own nodes using these as templates
5. ✅ Integrate with real hardware or Gazebo

---

**Remember**: This is pure ROS. No AI. Just physics, geometry, and control theory.

Master these fundamentals, then the world is yours.
