# ROS2 SLAM - Quick Reference Commands

## Environment Setup
```powershell
# Source ROS2 (do this in every new terminal)
C:\dev\ros2_humble\local_setup.ps1

# Source your workspace
.\install\setup.ps1
```

## Building
```powershell
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select slam_starter

# Clean build
Remove-Item -Recurse -Force build, install, log
colcon build --symlink-install
```

## Running SLAM

### Basic SLAM Launch
```powershell
ros2 launch slam_starter slam_launch.py
```

### SLAM with Monitor
```powershell
ros2 launch slam_starter slam_with_monitor.py
```

### Individual Nodes
```powershell
# SLAM node only
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file src/slam_starter/config/slam_toolbox_params.yaml

# Monitor node
ros2 run slam_starter slam_monitor

# Teleoperation (for testing with real/sim robot)
ros2 run slam_starter simple_teleop
```

## Visualization

### RViz2
```powershell
ros2 run rviz2 rviz2
```

In RViz2:
- Add → By topic → /map → Map
- Fixed Frame → map

## Diagnostics

### Check Topics
```powershell
# List all topics
ros2 topic list

# View topic info
ros2 topic info /scan
ros2 topic info /map

# Echo topic data
ros2 topic echo /scan
ros2 topic echo /map --once
```

### Check Transforms
```powershell
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

### Check Nodes
```powershell
# List running nodes
ros2 node list

# Node info
ros2 node info /slam_toolbox
```

## Map Operations

### Save Map
```powershell
# Save current map
ros2 run nav2_map_server map_saver_cli -f my_map

# Save with free threshold
ros2 run nav2_map_server map_saver_cli -f my_map --occ 0.65 --free 0.25
```

### Load Map (for localization)
```powershell
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml
```

## Working with Bag Files

### Record Data
```powershell
# Record specific topics
ros2 bag record /scan /tf /tf_static /odom

# Record all topics
ros2 bag record -a

# Record with specific name
ros2 bag record -o my_slam_session /scan /tf /tf_static
```

### Play Bag Files
```powershell
# Play at normal speed
ros2 bag play my_slam_session

# Play at half speed
ros2 bag play my_slam_session -r 0.5

# Play in loop
ros2 bag play my_slam_session -l
```

### Bag Info
```powershell
ros2 bag info my_slam_session
```

## Troubleshooting

### No Scan Data
```powershell
# Check if scan topic exists
ros2 topic list | Select-String scan

# Check scan message rate
ros2 topic hz /scan

# Echo one scan message
ros2 topic echo /scan --once
```

### Transform Issues
```powershell
# Check available frames
ros2 run tf2_ros tf2_monitor

# Debug specific transform
ros2 run tf2_ros tf2_echo map base_link
```

### SLAM Not Working
```powershell
# Check SLAM node is running
ros2 node list | Select-String slam

# Check SLAM parameters
ros2 param list /slam_toolbox

# Get specific parameter
ros2 param get /slam_toolbox scan_topic
```

## Advanced

### Switch to Localization Mode
Edit `config/slam_toolbox_params.yaml`:
```yaml
mode: localization  # change from 'mapping'
```

### Tune SLAM Parameters
Key parameters in `slam_toolbox_params.yaml`:
- `resolution`: 0.05 (map resolution in meters)
- `max_laser_range`: 20.0 (max range to use)
- `minimum_travel_distance`: 0.5 (meters before update)
- `minimum_travel_heading`: 0.5 (radians before update)

### Custom Scan Topic
```powershell
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p scan_topic:=/my_scan_topic
```

## Performance Monitoring

### CPU/Memory Usage
```powershell
# Windows Task Manager or:
Get-Process | Where-Object {$_.ProcessName -like "*ros*"} | Select-Object ProcessName, CPU, WS
```

### Topic Bandwidth
```powershell
ros2 topic bw /scan
ros2 topic bw /map
```

## Useful Aliases (Add to PowerShell Profile)

```powershell
# Edit: $PROFILE
function ros2-source { C:\dev\ros2_humble\local_setup.ps1; .\install\setup.ps1 }
function ros2-build { colcon build --symlink-install }
function ros2-slam { ros2 launch slam_starter slam_launch.py }
function ros2-viz { ros2 run rviz2 rviz2 }
```
