# Test SLAM with Fake Robot

This launch file demonstrates SLAM without needing a real robot or simulator!

## What it does

The `slam_test_fake_robot.py` launch file starts:
1. **Fake Scan Publisher** - Simulates a robot with a lidar moving in a circle in a 10x10m square room
2. **SLAM Toolbox** - The actual SLAM algorithm that builds the map
3. **SLAM Monitor** - Displays status information

## How to use

### Step 1: Build the package
```powershell
colcon build --symlink-install
```

### Step 2: Source the workspace
```powershell
.\install\setup.ps1
```

### Step 3: Launch the test
```powershell
ros2 launch slam_starter slam_test_fake_robot.py
```

You should see output from all three nodes showing:
- Fake robot publishing scan data
- SLAM building a map
- Monitor showing statistics

### Step 4: Visualize in RViz2

In a new terminal (with ROS2 and workspace sourced):
```powershell
ros2 run rviz2 rviz2
```

Configure RViz2:
1. Change **Fixed Frame** to `map`
2. Click **Add** → **By topic** → **/map** → **Map** → **OK**
3. Click **Add** → **By topic** → **/scan** → **LaserScan** → **OK**

You should see:
- A square room being mapped
- The laser scan data rotating around
- The map filling in over time

## What you're seeing

The fake robot:
- Moves in a 2-meter radius circle
- Publishes realistic laser scan data
- Simulates being in a 10x10 meter square room
- Publishes odometry and transforms

SLAM is:
- Processing the laser scans
- Building a map of the "room"
- Tracking the robot's position
- Publishing the map on `/map` topic

## Customizing

Edit [`fake_scan_publisher.py`](../slam_starter/fake_scan_publisher.py) to change:
- Room dimensions (`room_width`, `room_height`)
- Robot motion pattern (modify `publish_odometry` method)
- Scan characteristics
- Movement speed

## Next Steps

Once you verify SLAM works with the fake robot:
1. Try with a real robot
2. Try with Gazebo simulation
3. Try with recorded bag files
4. Tune SLAM parameters for your use case
