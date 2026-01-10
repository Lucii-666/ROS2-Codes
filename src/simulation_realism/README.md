# Simulation Realism Package

A comprehensive ROS2 package for physics tuning and sensor noise modeling in Ignition Gazebo simulations. This package provides realistic physics parameters, sensor noise models, and monitoring tools to create simulations that closely match real-world robot behavior.

## Features

### Physics Tuning
- **Accurate Mass and Inertia**: Realistic robot mass distribution and moment of inertia calculations
- **Contact Dynamics**: Tuned contact stiffness, damping, and friction coefficients
- **Fixed Time Step**: 1ms deterministic timestep for stable simulation
- **Material Properties**: Different friction values for wheels, casters, and ground surfaces
- **Joint Dynamics**: Proper damping and friction for wheel joints

### Sensor Noise Modeling

#### LiDAR
- Gaussian range noise (2cm stddev)
- Ray dropout modeling (2% dropout rate)
- Outlier detection simulation (1% outliers)
- Configurable range and resolution

#### Camera
- Gaussian pixel noise
- Lens distortion (Brown-Conrady model)
- Motion blur simulation
- Depth camera with range-dependent noise

#### IMU
- Gyroscope bias drift and random walk
- Accelerometer bias and noise
- Temperature-dependent drift simulation
- Realistic correlation times for bias

### Monitoring Tools
- Real-time sensor noise statistics
- Physics simulation performance metrics
- Contact stability monitoring
- Velocity tracking accuracy analysis

## Package Structure

```
simulation_realism/
├── config/
│   ├── physics_params.yaml          # Physics engine parameters
│   ├── sensor_noise_params.yaml     # Sensor noise configuration
│   └── rviz_params.yaml             # Visualization settings
├── launch/
│   ├── physics_tuned_simulation.launch.py  # Main simulation launch
│   └── sensor_test.launch.py               # Sensor testing only
├── urdf/
│   └── robot_with_physics.urdf      # Robot URDF with physics properties
├── worlds/
│   ├── physics_tuned_world.sdf      # Gazebo world with physics config
│   └── robot_with_sensors.sdf       # Robot SDF model with sensors
└── simulation_realism/
    ├── sensor_noise_monitor.py      # Sensor statistics node
    └── physics_stats_node.py        # Physics monitoring node
```

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
# Package already exists in your workspace
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select simulation_realism
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch Full Simulation

Launch the complete simulation with Gazebo, sensors, and monitoring:

```bash
ros2 launch simulation_realism physics_tuned_simulation.launch.py
```

This will start:
- Ignition Gazebo with physics-tuned world
- Robot with all sensors (LiDAR, Camera, Depth Camera, IMU)
- ROS2-Gazebo bridges for sensor topics
- Sensor noise monitoring node
- Physics statistics node
- RViz2 for visualization

### Launch Sensor Testing Only

For testing sensor noise without full Gazebo GUI:

```bash
ros2 launch simulation_realism sensor_test.launch.py
```

### Control the Robot

Use keyboard teleop or publish to cmd_vel:

```bash
# Install teleop_twist_keyboard if needed
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

Or publish directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### Monitor Sensor Data

View sensor topics:

```bash
# LiDAR
ros2 topic echo /scan

# Camera
ros2 topic echo /camera/image

# Depth
ros2 topic echo /camera/depth

# IMU
ros2 topic echo /imu

# Odometry
ros2 topic echo /odom
```

View sensor statistics:

```bash
# LiDAR statistics
ros2 topic echo /sensor_stats/lidar

# IMU statistics
ros2 topic echo /sensor_stats/imu
```

View physics statistics:

```bash
# Real-time factor
ros2 topic echo /physics_stats/rtf

# Stability metrics
ros2 topic echo /physics_stats/stability
```

## Parameter Tuning Guide

### Physics Parameters

Edit [config/physics_params.yaml](config/physics_params.yaml):

#### Time Step Tuning
```yaml
physics:
  max_step_size: 0.001  # Smaller = more accurate but slower
  real_time_factor: 1.0  # Target real-time performance
```

**Guidelines:**
- Use 0.001s (1ms) for high-precision robotics
- Use 0.01s (10ms) for faster-than-real-time simulation
- Adjust `real_time_factor` based on your hardware

#### Contact Dynamics
```yaml
contact:
  kp: 1.0e6   # Higher = stiffer contacts
  kd: 1.0e3   # Higher = more damping
```

**Guidelines:**
- Increase `kp` if objects penetrate each other
- Increase `kd` if contacts are bouncy/unstable
- Balance between stability and realism

#### Friction Coefficients
```yaml
friction:
  wheel_mu1: 1.2  # Wheel-ground friction
  caster_mu1: 0.05  # Caster-ground friction
```

**Guidelines:**
- Typical rubber-concrete: 0.8-1.2
- Metal-metal: 0.2-0.4
- Ice/low friction: 0.05-0.1

### Sensor Noise Parameters

Edit [config/sensor_noise_params.yaml](config/sensor_noise_params.yaml):

#### LiDAR Noise
```yaml
lidar:
  noise_stddev: 0.02  # 2cm standard deviation
  dropout_probability: 0.02  # 2% dropout rate
```

**Real-world reference:**
- High-end LiDAR: 0.01-0.02m stddev
- Mid-range LiDAR: 0.02-0.05m stddev
- Low-cost LiDAR: 0.05-0.1m stddev

#### Camera Noise
```yaml
camera:
  noise_stddev: 0.007  # Pixel noise
  distortion:
    k1: -0.02  # Radial distortion
```

**Real-world reference:**
- High-quality camera: 0.003-0.007 noise
- Webcam: 0.01-0.02 noise
- Distortion varies by lens quality

#### IMU Noise
```yaml
imu:
  gyro:
    noise_stddev: 0.009  # ~0.5 deg/s
    bias_mean: 0.00008726646  # ~0.005 deg/s
  accel:
    noise_stddev: 0.017  # m/s²
    bias_mean: 0.05  # m/s²
```

**Real-world reference:**
- Consumer IMU: 0.01-0.02 rad/s gyro noise
- Industrial IMU: 0.001-0.01 rad/s
- MEMS accelerometer: 0.01-0.1 m/s² noise

## Physics Validation

### Check Real-Time Factor

```bash
ros2 topic echo /physics_stats/rtf
```

- RTF = 1.0: Running at real-time speed
- RTF < 1.0: Simulation is slower than real-time
- RTF > 1.0: Simulation is faster than real-time

**Troubleshooting slow simulation:**
1. Increase `max_step_size` to 0.01s
2. Reduce sensor update rates
3. Disable camera visualization
4. Use simpler collision geometry

### Check Contact Stability

Monitor the physics stability metrics:

```bash
ros2 topic echo /physics_stats/stability
```

- `jerk`: Rate of acceleration change (lower is better)
- `drift`: Velocity tracking error (lower is better)

**If contacts are unstable:**
1. Increase contact `kd` (damping)
2. Decrease `max_vel`
3. Increase `min_depth`

### Validate Sensor Noise

Check that sensor noise matches expected values:

```bash
ros2 run simulation_realism sensor_noise_monitor
```

Compare reported statistics with configured parameters.

## Reinforcement Learning Integration

This package is designed for RL and sim-to-real transfer:

### Benefits for RL

1. **Realistic Dynamics**: Prevents overfitting to perfect physics
2. **Sensor Noise**: Forces robust perception strategies
3. **Contact Modeling**: Handles pushing, grasping realistically
4. **Deterministic Simulation**: Fixed timestep for reproducibility

### Domain Randomization

Extend this package by randomizing:
- Mass properties (±10%)
- Friction coefficients (±20%)
- Sensor noise levels (±50%)
- Contact parameters

### Sim-to-Real Transfer

Key parameters to match real robot:
1. Measure actual robot mass and CoM
2. Calibrate sensor noise from real data
3. Measure friction coefficients
4. Tune contact dynamics by observation

## Advanced Topics

### Custom Robot Models

To use your own robot:

1. Create URDF with proper inertia tensors
2. Add Gazebo tags for friction and contact
3. Include sensor plugins with noise
4. Reference in launch file

See [urdf/robot_with_physics.urdf](urdf/robot_with_physics.urdf) as template.

### Multi-Robot Simulation

Spawn multiple robots with different noise profiles:

```python
# In launch file
for i in range(num_robots):
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', f'robot_{i}', '-file', robot_sdf],
        namespace=f'robot_{i}'
    )
```

### Adding Custom Sensors

1. Add sensor to SDF model
2. Configure noise in sensor block
3. Add ROS2-Gazebo bridge
4. Update monitoring node

## Troubleshooting

### Robot Falls Through Ground
- Increase ground `kp` (stiffness)
- Check robot mass is not too heavy
- Verify collision geometry exists

### Wheels Slip Excessively
- Increase wheel friction (`mu1`, `mu2`)
- Decrease ground friction
- Check wheel joint damping

### Simulation Crashes
- Reduce timestep (increase `max_step_size`)
- Simplify collision geometry
- Check for NaN in sensor data

### Sensors Not Publishing
- Verify ROS2-Gazebo bridge is running
- Check topic names match
- Ensure sensors are enabled in SDF

## Performance Optimization

### For Faster Simulation
1. Use simpler collision shapes (spheres, boxes)
2. Reduce sensor update rates
3. Disable visualization
4. Increase timestep to 0.01s

### For Higher Accuracy
1. Decrease timestep to 0.0001s
2. Use convex mesh collisions
3. Increase solver iterations
4. Use finer sensor resolution

## Citation

If you use this package in your research:

```bibtex
@software{simulation_realism_2026,
  title={Simulation Realism: Physics Tuning and Sensor Noise for ROS2},
  author={Your Name},
  year={2026},
  url={https://github.com/yourusername/simulation_realism}
}
```

## License

Apache 2.0 License

## Contributing

Contributions welcome! Areas for improvement:
- Additional sensor types (radar, sonar)
- Temperature effects on sensors
- Wear modeling for joints
- Wind and environmental disturbances

## Support

For issues and questions:
- GitHub Issues: [your-repo-url]
- ROS Answers: Tag with `simulation_realism`
- Documentation: See package README files

## References

- Ignition Gazebo Documentation: https://gazebosim.org/
- ROS2 Documentation: https://docs.ros.org/
- Sensor Noise Models: IEEE Robotics papers
- Physics Engine Tuning: ODE/Bullet documentation
