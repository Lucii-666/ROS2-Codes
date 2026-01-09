# Perception Demos - Quick Start Guide

## 🚀 Quick Installation

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Build the perception package
colcon build --packages-select perception_demos

# Source the workspace
source install/setup.bash

# Or on Windows
call install\setup.bat
```

## ⚡ Quick Launch Examples

### 1. Test USB Camera (5 seconds)
```bash
ros2 run perception_demos usb_camera_node
```

### 2. Test Object Detection (1 minute)
```bash
# Terminal 1: Launch detection pipeline
ros2 launch perception_demos yolo_detection_launch.py

# Terminal 2: View results
ros2 run rqt_image_view rqt_image_view /yolo/visualization
```

### 3. Test Marker Detection (1 minute)
```bash
# Generate markers first
ros2 run perception_demos marker_generator_node

# Print markers from ./markers/ directory
# Then run:
ros2 launch perception_demos marker_detection_launch.py
```

### 4. Test All Perception Features (2 minutes)
```bash
ros2 launch perception_demos full_perception_launch.py
```

## 📊 Verify Installation

```bash
# List all 30 executables
ros2 pkg executables perception_demos

# Expected output (30 nodes):
# perception_demos apriltag_detector_node
# perception_demos aruco_detector_node
# perception_demos camera_calibration_node
# ... (27 more)
```

## 🎯 Individual Node Testing

### Camera Nodes
```bash
ros2 run perception_demos usb_camera_node
ros2 run perception_demos realsense_camera_node
ros2 run perception_demos zed_camera_node
ros2 run perception_demos multi_camera_node
ros2 run perception_demos camera_calibration_node
```

### OpenCV Nodes
```bash
ros2 run perception_demos image_subscriber
ros2 run perception_demos image_filter_node
ros2 run perception_demos edge_detection_node
ros2 run perception_demos color_detection_node --ros-args -p target_color:=red
ros2 run perception_demos image_blend_node
ros2 run perception_demos video_recorder_node
```

### Object Detection Nodes
```bash
ros2 run perception_demos yolo_detector_node
ros2 run perception_demos yolo_webcam_node
ros2 run perception_demos object_tracker_node
ros2 run perception_demos roi_extractor_node
ros2 run perception_demos detection_visualizer_node
ros2 run perception_demos detection_filter_node
```

### Marker Nodes
```bash
ros2 run perception_demos apriltag_detector_node
ros2 run perception_demos aruco_detector_node
ros2 run perception_demos marker_pose_estimator_node
ros2 run perception_demos marker_generator_node
ros2 run perception_demos multi_marker_tracker_node
```

### Depth & Point Cloud Nodes
```bash
ros2 run perception_demos depth_image_processor_node
ros2 run perception_demos pointcloud_generator_node
ros2 run perception_demos pointcloud_filter_node
ros2 run perception_demos obstacle_detection_node
ros2 run perception_demos object_3d_locator_node
```

### Advanced Perception Nodes
```bash
ros2 run perception_demos face_detection_node
ros2 run perception_demos pose_estimation_node
ros2 run perception_demos semantic_segmentation_node
```

## 🔍 View Topics

```bash
# List all topics
ros2 topic list

# View camera images
ros2 topic echo /camera/image_raw

# View detections
ros2 topic echo /yolo/detections

# View marker poses
ros2 topic echo /aruco/poses
```

## 📹 Visualize with RViz2

```bash
rviz2

# Add displays:
# - Image -> /camera/image_raw
# - Image -> /yolo/visualization
# - PointCloud2 -> /pointcloud
# - TF -> Show marker frames
```

## 🐛 Troubleshooting

### Camera not found
```bash
# List available cameras
ls /dev/video*

# Or on Windows
Get-PnpDevice -Class Camera
```

### Permission denied
```bash
sudo usermod -a -G video $USER
# Then logout and login
```

### Module not found
```bash
pip install opencv-python opencv-contrib-python numpy
```

## 📚 Next Steps

1. Read [README.md](README.md) for detailed documentation
2. Check [DEVELOPMENT_SUMMARY.md](DEVELOPMENT_SUMMARY.md) for architecture
3. Explore individual node parameters:
   ```bash
   ros2 run perception_demos <node_name> --ros-args --help
   ```
4. Modify [config/perception_params.yaml](config/perception_params.yaml) for your needs
5. Create custom launch files based on examples in [launch/](launch/)

## 🎉 You're Ready!

You now have 30 perception nodes ready to use for:
- Camera integration (5 nodes)
- OpenCV processing (6 nodes)
- Object detection (6 nodes)
- Marker tracking (5 nodes)
- Depth perception (5 nodes)
- Advanced AI vision (3 nodes)

Happy perceiving! 🤖👁️
