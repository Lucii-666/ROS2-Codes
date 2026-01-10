# Physics and Sensor Parameter Reference

Complete reference for all tunable parameters in the simulation_realism package.

## Table of Contents
- [Physics Parameters](#physics-parameters)
- [Sensor Parameters](#sensor-parameters)
- [Monitoring Parameters](#monitoring-parameters)

---

## Physics Parameters

File: `config/physics_params.yaml`

### Time Step Configuration

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `max_step_size` | float | 0.001 | 0.0001-0.01 | Fixed simulation timestep (seconds) |
| `real_time_factor` | float | 1.0 | 0.1-10.0 | Target simulation speed vs real-time |
| `real_time_update_rate` | float | 1000.0 | 100-10000 | Physics update frequency (Hz) |

**Notes:**
- Smaller timestep = more accurate but slower
- RTF < 1.0 means simulation can't keep up with real-time
- Update rate should be 1/max_step_size

### Contact Dynamics

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `contact.kp` | float | 1.0e6 | 1e4-1e8 | Contact stiffness (N/m) |
| `contact.kd` | float | 1.0e3 | 1e2-1e5 | Contact damping (N*s/m) |
| `contact.min_depth` | float | 0.001 | 0.0001-0.01 | Minimum penetration depth (m) |
| `contact.max_vel` | float | 0.01 | 0.001-0.1 | Maximum contact correction velocity (m/s) |

**Tuning Guide:**
- Increase `kp` if objects penetrate surfaces
- Increase `kd` if contacts bounce unrealistically
- Decrease `max_vel` if simulation is unstable

### Robot Mass Properties

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `robot.base_mass` | float | 15.0 | kg | Main robot body mass |
| `robot.wheel_mass` | float | 0.8 | kg | Mass per wheel |
| `robot.sensor_mount_mass` | float | 0.5 | kg | Sensor mount mass |

**Real-world Matching:**
- Weigh your actual robot
- Measure center of mass location
- Calculate moment of inertia from CAD

### Friction Coefficients

| Parameter | Type | Default | Range | Material Example |
|-----------|------|---------|-------|------------------|
| `friction.base_mu1` | float | 0.8 | 0.0-2.0 | Robot body on ground |
| `friction.base_mu2` | float | 0.8 | 0.0-2.0 | Secondary friction direction |
| `friction.wheel_mu1` | float | 1.2 | 0.5-2.0 | Rubber tire on concrete |
| `friction.wheel_mu2` | float | 1.2 | 0.5-2.0 | Rubber tire lateral |
| `friction.caster_mu1` | float | 0.05 | 0.01-0.2 | Low-friction caster |
| `friction.caster_mu2` | float | 0.05 | 0.01-0.2 | Caster lateral |
| `friction.ground_mu1` | float | 0.8 | 0.3-1.5 | Ground surface |
| `friction.ground_mu2` | float | 0.8 | 0.3-1.5 | Ground surface |

**Material Reference:**
- Rubber on dry concrete: 0.8-1.2
- Rubber on wet surface: 0.4-0.7
- Metal on metal: 0.2-0.4
- Teflon/plastic: 0.04-0.15
- Ice: 0.02-0.05

### Dynamics

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `dynamics.wheel_damping` | float | 0.1 | N*m*s/rad | Rotational damping |
| `dynamics.wheel_friction` | float | 0.1 | N*m | Static friction torque |

### Differential Drive

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `diff_drive.wheel_separation` | float | 0.45 | m | Distance between wheels |
| `diff_drive.wheel_radius` | float | 0.1 | m | Wheel radius |
| `diff_drive.odom_publish_frequency` | float | 50.0 | Hz | Odometry rate |

---

## Sensor Parameters

File: `config/sensor_noise_params.yaml`

### LiDAR Configuration

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `lidar.update_rate` | float | 10.0 | 5-100 Hz | Scan frequency |
| `lidar.samples` | int | 360 | 180-1080 | Number of beams |
| `lidar.range_min` | float | 0.12 | 0.05-0.5 m | Minimum detection range |
| `lidar.range_max` | float | 10.0 | 5-100 m | Maximum detection range |
| `lidar.resolution` | float | 0.015 | 0.001-0.1 m | Range measurement resolution |

**Noise Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar.noise_mean` | float | 0.0 | Mean of Gaussian noise (m) |
| `lidar.noise_stddev` | float | 0.02 | Standard deviation (m) |
| `lidar.dropout_probability` | float | 0.02 | Fraction of missed returns (0-1) |
| `lidar.outlier_probability` | float | 0.01 | Fraction of outlier readings (0-1) |
| `lidar.outlier_range_min` | float | 0.5 | Outlier minimum range (m) |
| `lidar.outlier_range_max` | float | 9.5 | Outlier maximum range (m) |

**Real Sensor Examples:**
- Hokuyo UTM-30LX: 0.01m stddev, 30m range
- RPLIDAR A1: 0.05m stddev, 12m range
- Velodyne VLP-16: 0.03m stddev, 100m range

### Camera Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera.update_rate` | float | 30.0 | Frame rate (FPS) |
| `camera.width` | int | 640 | Image width (pixels) |
| `camera.height` | int | 480 | Image height (pixels) |
| `camera.horizontal_fov` | float | 1.047 | Field of view (radians, ~60°) |

**Noise Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera.noise_mean` | float | 0.0 | Pixel noise mean (normalized) |
| `camera.noise_stddev` | float | 0.007 | Pixel noise std (normalized 0-1) |

**Distortion Model (Brown-Conrady):**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `distortion.k1` | float | -0.02 | Radial distortion coefficient 1 |
| `distortion.k2` | float | 0.01 | Radial distortion coefficient 2 |
| `distortion.k3` | float | -0.001 | Radial distortion coefficient 3 |
| `distortion.p1` | float | 0.001 | Tangential distortion 1 |
| `distortion.p2` | float | -0.001 | Tangential distortion 2 |

**Calibration:**
- Use `camera_calibration` package to measure real distortion
- Typical webcam k1: -0.1 to -0.3
- High-quality lens k1: -0.01 to -0.05

### Depth Camera Configuration

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `depth_camera.update_rate` | float | 30.0 | Hz | Depth frame rate |
| `depth_camera.noise_mean` | float | 0.0 | m | Depth noise mean |
| `depth_camera.noise_stddev` | float | 0.01 | m | Depth noise std |
| `depth_camera.min_depth` | float | 0.1 | m | Minimum depth |
| `depth_camera.max_depth` | float | 10.0 | m | Maximum depth |

**Real Sensor Examples:**
- Intel RealSense D435: 0.01m @ 1m, 0.05m @ 5m
- Azure Kinect: 0.017m @ 1m, 0.05m @ 3m
- Structure Core: 0.005m @ 1m

### IMU Configuration

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `imu.update_rate` | float | 100.0 | Hz | IMU sample rate |

**Gyroscope (Angular Velocity):**

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `gyro.noise_mean` | float | 0.0 | rad/s | Mean of noise |
| `gyro.noise_stddev` | float | 0.009 | rad/s | White noise (~0.5°/s) |
| `gyro.bias_mean` | float | 8.73e-5 | rad/s | Static bias (~0.005°/s) |
| `gyro.bias_stddev` | float | 8.73e-5 | rad/s | Bias variation |
| `gyro.dynamic_bias_stddev` | float | 4.36e-5 | rad/s | Bias drift rate |
| `gyro.dynamic_bias_correlation_time` | float | 1000.0 | s | Drift time constant |

**Accelerometer (Linear Acceleration):**

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `accel.noise_mean` | float | 0.0 | m/s² | Mean of noise |
| `accel.noise_stddev` | float | 0.017 | m/s² | White noise |
| `accel.bias_mean` | float | 0.05 | m/s² | Static bias |
| `accel.bias_stddev` | float | 0.01 | m/s² | Bias variation |
| `accel.dynamic_bias_stddev` | float | 0.005 | m/s² | Bias drift rate |
| `accel.dynamic_bias_correlation_time` | float | 300.0 | s | Drift time constant |

**Real IMU Examples:**
- MPU6050 (MEMS): 0.05 rad/s noise, high drift
- BMI088 (mid-range): 0.01 rad/s noise, moderate drift
- ADIS16495 (tactical): 0.003 rad/s noise, low drift
- Xsens MTi (navigation): 0.001 rad/s noise, very low drift

**Conversion:**
- 1 deg/s = 0.0174533 rad/s
- 1 g = 9.81 m/s²

---

## Monitoring Parameters

File: `config/sensor_noise_params.yaml`

### Statistics Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `monitoring.enable_lidar_stats` | bool | true | Enable LiDAR monitoring |
| `monitoring.enable_imu_stats` | bool | true | Enable IMU monitoring |
| `monitoring.enable_odom_stats` | bool | true | Enable odometry monitoring |
| `monitoring.enable_camera_stats` | bool | false | Enable camera monitoring (expensive) |
| `monitoring.window_size` | int | 100 | Number of samples for statistics |
| `monitoring.publish_rate` | float | 1.0 | Statistics publish rate (Hz) |

---

## Parameter Tuning Workflow

### 1. Match Real Hardware

```yaml
# Measure your real robot
robot:
  base_mass: <measured_mass_kg>
  
# Test real sensors
lidar:
  noise_stddev: <measured_stddev>
```

### 2. Tune for Stability

```yaml
# If simulation unstable
contact:
  kd: 2.0e3  # Increase damping
  max_vel: 0.005  # Reduce max velocity
```

### 3. Optimize Performance

```yaml
# If simulation too slow
physics:
  max_step_size: 0.005  # Increase timestep
  
lidar:
  update_rate: 5.0  # Reduce sensor rate
```

### 4. Validate Results

```bash
# Check statistics match configuration
ros2 topic echo /sensor_stats/lidar

# Monitor physics performance
ros2 topic echo /physics_stats/rtf
```

---

## Example Configurations

### High-Accuracy Simulation

```yaml
physics:
  max_step_size: 0.0005  # 0.5ms
  
lidar:
  noise_stddev: 0.01
  samples: 720
```

### Fast Simulation (RL Training)

```yaml
physics:
  max_step_size: 0.01  # 10ms
  
lidar:
  noise_stddev: 0.03
  samples: 180
  update_rate: 5.0
```

### Sim-to-Real Transfer

```yaml
# Measure and match real robot
robot:
  base_mass: 14.8  # Measured value
  
friction:
  wheel_mu1: 0.95  # Calibrated from tests
  
lidar:
  noise_stddev: 0.023  # From sensor datasheet
```

---

## Units Reference

| Quantity | Unit | Symbol |
|----------|------|--------|
| Length | meter | m |
| Mass | kilogram | kg |
| Time | second | s |
| Force | newton | N |
| Torque | newton-meter | N*m |
| Angle | radian | rad |
| Angular velocity | radian/second | rad/s |
| Linear velocity | meter/second | m/s |
| Acceleration | meter/second² | m/s² |
| Frequency | hertz | Hz |
| Stiffness | newton/meter | N/m |
| Damping | newton*second/meter | N*s/m |

---

## Validation Checklist

- [ ] Robot doesn't fall through ground
- [ ] Wheels provide proper traction
- [ ] Real-time factor > 0.8
- [ ] Sensor noise matches configuration
- [ ] Contacts are stable (no jittering)
- [ ] Odometry drift is realistic
- [ ] IMU bias drift is present
- [ ] LiDAR dropouts occur occasionally

---

For more information, see:
- [README.md](README.md) - Full package documentation
- [QUICKSTART.md](QUICKSTART.md) - Getting started guide
