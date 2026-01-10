# Quick Start Guide - Simulation Realism

Get up and running with realistic physics simulation in 5 minutes!

## Prerequisites

- ROS2 Humble or later
- Ignition Gazebo (Fortress or later)
- Ubuntu 22.04 or compatible system

## Installation

```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Package should already be here as simulation_realism

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select simulation_realism

# Source
source install/setup.bash
```

## Quick Test

### 1. Launch the Simulation

```bash
ros2 launch simulation_realism physics_tuned_simulation.launch.py
```

This opens:
- Gazebo with a tuned physics world
- Robot with LiDAR, camera, IMU
- Monitoring nodes for statistics

### 2. Control the Robot

In a new terminal:

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

Or use keyboard control:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. View Sensor Data

```bash
# View LiDAR scan
ros2 topic echo /scan --once

# View IMU data
ros2 topic echo /imu --once

# View camera image info
ros2 topic info /camera/image
```

### 4. Monitor Statistics

```bash
# Check sensor noise statistics
ros2 topic echo /sensor_stats/lidar

# Check physics performance
ros2 topic echo /physics_stats/rtf
```

## What You Should See

1. **Gazebo Window**: Robot in a world with obstacles
2. **RViz Window**: Visualization of LiDAR, odometry, robot model
3. **Terminal Output**: Sensor statistics and physics metrics

### Expected Statistics

- **LiDAR**: ~2cm noise stddev, 2% dropouts
- **IMU**: ~0.5 deg/s gyro noise, accelerometer bias
- **Real-time Factor**: ~0.8-1.0 (depending on hardware)

## Quick Tuning

### Make Robot Faster

Edit `config/physics_params.yaml`:
```yaml
physics:
  max_step_size: 0.01  # 10ms instead of 1ms
```

### Reduce Sensor Noise

Edit `config/sensor_noise_params.yaml`:
```yaml
lidar:
  noise_stddev: 0.01  # Reduce from 0.02
```

### Change Robot Behavior

Try different friction:
```yaml
friction:
  wheel_mu1: 0.5  # More slippery wheels
```

## Common First-Run Issues

### "Package not found"
```bash
# Make sure you sourced the workspace
source ~/ros2_ws/install/setup.bash
```

### "Failed to load world"
```bash
# Check Gazebo is installed
gz sim --version

# Install if missing
sudo apt install ros-humble-ros-gz
```

### "No sensor data"
```bash
# Verify bridges are running
ros2 node list | grep bridge
```

### Simulation too slow
```bash
# Edit physics_params.yaml and increase timestep
# OR run in headless mode:
ros2 launch simulation_realism physics_tuned_simulation.launch.py headless:=true
```

## Next Steps

1. **Read full README.md** for detailed parameter tuning
2. **Customize robot URDF** with your own robot model
3. **Add sensors** by editing the SDF files
4. **Integrate with your navigation stack**
5. **Use for reinforcement learning** training

## Example Commands

```bash
# List all topics
ros2 topic list

# Visualize LiDAR in RViz
ros2 run rviz2 rviz2

# Record sensor data
ros2 bag record /scan /imu /odom

# Check node status
ros2 node info /sensor_noise_monitor
```

## Help

- **Full documentation**: See README.md
- **Configuration files**: Check config/ directory
- **Troubleshooting**: See README.md Troubleshooting section

## Success Indicators

✅ Gazebo opens without errors  
✅ Robot model loads and sits on ground  
✅ LiDAR shows range data with noise  
✅ Robot responds to cmd_vel commands  
✅ Statistics nodes publish data  
✅ Real-time factor > 0.5  

If all these work, you're ready to use the package!

Happy simulating! 🤖
