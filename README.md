# ROS2 SLAM Complete Package

A comprehensive ROS2 SLAM (Simultaneous Localization and Mapping) workspace using `slam_toolbox` for Windows.

## 📁 Package Structure

```
G:\ROS\
├── src\
│   └── slam_starter\              # Main SLAM package
│       ├── config\
│       │   └── slam_toolbox_params.yaml    # SLAM configuration
│       ├── launch\
│       │   ├── slam_launch.py              # Basic SLAM launch
│       │   ├── slam_with_monitor.py        # SLAM + monitoring
│       │   └── slam_test_fake_robot.py     # Complete test demo
│       ├── slam_starter\
│       │   ├── __init__.py
│       │   ├── slam_monitor.py             # Monitor node
│       │   ├── simple_teleop.py            # Teleoperation
│       │   └── fake_scan_publisher.py      # Fake robot for testing
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       └── TEST_README.md                  # Testing guide
├── download_dependencies.ps1               # Dependency downloader
├── SETUP_GUIDE.md                          # Detailed setup instructions
├── COMMANDS.md                             # Quick command reference
└── README.md                               # This file
```

## 🚀 Quick Start

### 1. Prerequisites
- ROS2 Humble (or compatible version) installed on Windows
- Git installed
- Visual Studio with C++ tools
- Colcon build tools

### 2. Download Dependencies
```powershell
.\download_dependencies.ps1
```

### 3. Build
```powershell
colcon build --symlink-install
```

### 4. Test with Fake Robot (No real hardware needed!)
```powershell
.\install\setup.ps1
ros2 launch slam_starter slam_test_fake_robot.py
```

### 5. Visualize
In a new terminal:
```powershell
ros2 run rviz2 rviz2
```
- Set Fixed Frame to `map`
- Add `/map` and `/scan` topics

## 📚 Documentation

- **[SETUP_GUIDE.md](SETUP_GUIDE.md)** - Complete installation and setup instructions
- **[COMMANDS.md](COMMANDS.md)** - Quick reference for all ROS2 commands
- **[TEST_README.md](src/slam_starter/TEST_README.md)** - How to test without a robot

## 🎯 Features

### Core Functionality
- ✅ SLAM Toolbox integration (online async mapping)
- ✅ Configurable parameters
- ✅ Map saving/loading
- ✅ Localization mode support

### Testing & Development Tools
- ✅ **Fake Robot Simulator** - Test SLAM without hardware
- ✅ **SLAM Monitor** - Real-time status display
- ✅ Multiple launch configurations

### Included Nodes

1. **slam_toolbox** - The SLAM algorithm
2. **slam_monitor** - Monitors map and scan data
3. **fake_scan_publisher** - Simulates robot with lidar

## 🎮 Usage Examples

### Basic SLAM (with real robot or simulator)
```powershell
ros2 launch slam_starter slam_launch.py
```

### Test Without Robot
```powershell
# Complete demo with fake robot
ros2 launch slam_starter slam_test_fake_robot.py

# Just the fake robot (test separately)
ros2 run slam_starter fake_scan_publisher
```

### With Monitoring
```powershell
ros2 launch slam_starter slam_with_monitor.py
```

### Save Your Map
```powershell
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 🔧 Configuration

Edit [`config/slam_toolbox_params.yaml`](src/slam_starter/config/slam_toolbox_params.yaml):

```yaml
# Key parameters to tune
resolution: 0.05              # Map resolution (meters per pixel)
max_laser_range: 20.0         # Maximum laser range to use
scan_topic: /scan             # Your lidar topic
mode: mapping                 # or 'localization'
do_loop_closing: true         # Enable loop closure
```

## 📊 Monitoring & Debugging

### Check Status
```powershell
# List topics
ros2 topic list

# Check scan data
ros2 topic hz /scan
ros2 topic echo /scan --once

# View transforms
ros2 run tf2_ros tf2_echo map odom
```

### Run Monitor
```powershell
ros2 run slam_starter slam_monitor
```

## 🎓 Learning Path

1. **Start Here**: Run `slam_test_fake_robot.py` to see SLAM in action
2. **Understand**: Read through the code and config files
3. **Experiment**: Modify parameters and see the effects
4. **Apply**: Use with your own robot or simulator
5. **Optimize**: Tune parameters for your environment

## 🤖 Using with Real Hardware

### Requirements
Your robot needs to publish:
- **LaserScan** on `/scan` (or configure different topic)
- **Transforms**: `odom → base_link`
- **Odometry** (optional but recommended)

### Steps
1. Start your robot's drivers
2. Verify topics: `ros2 topic list`
3. Launch SLAM: `ros2 launch slam_starter slam_launch.py`
4. Move your robot (manually or with teleop)
5. Watch the map build in RViz2
6. Save the map when done

## 📦 Dependencies

Core ROS2 packages used:
- `slam_toolbox` - SLAM implementation
- `nav2_map_server` - Map saving/loading
- `sensor_msgs` - Laser scan messages
- `nav_msgs` - Occupancy grid (map)
- `geometry_msgs` - Poses and transforms
- `tf2_ros` - Transform broadcasting

## 🐛 Troubleshooting

### Build Errors
```powershell
# Clean build
Remove-Item -Recurse -Force build, install, log
colcon build --symlink-install
```

### No Map Appearing
- Check scan topic: `ros2 topic echo /scan --once`
- Verify transforms: `ros2 run tf2_ros tf2_monitor`
- Check SLAM is running: `ros2 node list`

### Performance Issues
- Reduce scan frequency in config
- Decrease map resolution
- Limit laser range

## 📖 Additional Resources

- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2](https://navigation.ros.org/)

## 🤝 Contributing

Feel free to:
- Add new launch configurations
- Improve documentation
- Share parameter configurations
- Report issues

## 📄 License

Apache 2.0

---

**Need help?** Check the detailed guides:
- Installation issues → [SETUP_GUIDE.md](SETUP_GUIDE.md)
- Command reference → [COMMANDS.md](COMMANDS.md)
- Testing → [TEST_README.md](src/slam_starter/TEST_README.md)
