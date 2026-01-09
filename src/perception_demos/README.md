# Perception Demos - ROS2 Computer Vision Package

Comprehensive perception examples for ROS2 covering cameras, OpenCV, object detection, markers, depth sensing, and AI-powered vision.

## 🎯 Features

### Camera Drivers (5 nodes)
- **usb_camera_node** - USB webcam capture and publishing
- **realsense_camera_node** - Intel RealSense depth camera integration
- **zed_camera_node** - Stereolabs ZED stereo camera
- **multi_camera_node** - Multi-camera synchronization
- **camera_calibration_node** - Camera calibration with chessboard

### OpenCV Basics (6 nodes)
- **image_subscriber** - Image display with OpenCV
- **image_filter_node** - Various filters (Gaussian, median, bilateral, morphological)
- **edge_detection_node** - Edge detection (Canny, Sobel, Laplacian, Scharr)
- **color_detection_node** - HSV-based color detection
- **image_blend_node** - Image blending and compositing
- **video_recorder_node** - Video recording to disk

### Object Detection (6 nodes)
- **yolo_detector_node** - YOLO object detection
- **yolo_webcam_node** - Real-time YOLO with webcam
- **object_tracker_node** - Multi-object tracking
- **roi_extractor_node** - Extract regions of interest
- **detection_visualizer_node** - Advanced detection visualization
- **detection_filter_node** - Filter detections by criteria

### Markers (5 nodes)
- **apriltag_detector_node** - AprilTag fiducial detection
- **aruco_detector_node** - ArUco marker detection
- **marker_pose_estimator_node** - 6DOF pose estimation
- **marker_generator_node** - Generate printable markers
- **multi_marker_tracker_node** - Track multiple markers

### Depth & Point Clouds (5 nodes)
- **depth_image_processor_node** - Process depth images
- **pointcloud_generator_node** - RGB-D to point cloud
- **pointcloud_filter_node** - Filter point clouds
- **obstacle_detection_node** - Detect obstacles from depth
- **object_3d_locator_node** - 3D object localization

### Advanced Perception (3 nodes)
- **face_detection_node** - Face detection (Haar cascades)
- **pose_estimation_node** - Human pose estimation
- **semantic_segmentation_node** - Pixel-wise segmentation

## 📦 Installation

### Dependencies
```bash
# ROS2 packages
sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-vision-msgs

# Python packages
pip install opencv-python opencv-contrib-python numpy

# Optional (for full functionality)
pip install pyrealsense2  # Intel RealSense
pip install pyzed  # Stereolabs ZED
pip install apriltag  # AprilTag detection
pip install ultralytics  # YOLOv8
pip install torch torchvision  # PyTorch for YOLO
pip install mediapipe  # Pose estimation
pip install scikit-learn  # Clustering for obstacles
```

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select perception_demos
source install/setup.bash
```

## 🚀 Usage

### Launch Examples

#### Basic Camera
```bash
ros2 launch perception_demos camera_launch.py
```

#### YOLO Object Detection
```bash
ros2 launch perception_demos yolo_detection_launch.py
```

#### ArUco Marker Detection
```bash
ros2 launch perception_demos marker_detection_launch.py
```

#### Depth Perception
```bash
ros2 launch perception_demos depth_perception_launch.py
```

#### Full Perception Pipeline
```bash
ros2 launch perception_demos full_perception_launch.py
```

### Individual Nodes

#### USB Camera
```bash
ros2 run perception_demos usb_camera_node --ros-args -p camera_index:=0
```

#### YOLO Detection
```bash
ros2 run perception_demos yolo_detector_node
```

#### Color Detection
```bash
ros2 run perception_demos color_detection_node --ros-args -p target_color:=red
```

#### Edge Detection
```bash
ros2 run perception_demos edge_detection_node --ros-args -p algorithm:=canny
```

#### ArUco Detection
```bash
ros2 run perception_demos aruco_detector_node
```

#### Face Detection
```bash
ros2 run perception_demos face_detection_node
```

#### Video Recording
```bash
ros2 run perception_demos video_recorder_node

# Start/stop recording
ros2 topic pub /video_recorder/record std_msgs/Bool "data: true"
ros2 topic pub /video_recorder/record std_msgs/Bool "data: false"
```

### Generate Markers
```bash
ros2 run perception_demos marker_generator_node --ros-args \
    -p marker_type:=aruco \
    -p marker_ids:=[0,1,2,3,4] \
    -p output_directory:=./markers
```

## 📊 Topics

### Published Topics
- `/camera/image_raw` - Raw camera images
- `/camera/camera_info` - Camera calibration info
- `/yolo/detections` - Object detections
- `/yolo/visualization` - Annotated detection images
- `/edges/image` - Edge detection results
- `/color_detection/mask` - Color detection mask
- `/aruco/poses` - Marker poses
- `/depth/processed` - Processed depth images
- `/pointcloud` - Generated point clouds
- `/faces/poses` - Detected face locations
- `/obstacles/poses` - Detected obstacles

### Subscribed Topics
- `/camera/image_raw` - Input images
- `/camera/depth/image_raw` - Input depth images
- `/yolo/detections` - Detection messages for downstream processing

## ⚙️ Parameters

### Camera Node
- `camera_index` (int, default: 0) - Camera device index
- `frame_rate` (float, default: 30.0) - Capture frame rate
- `width` (int, default: 640) - Image width
- `height` (int, default: 480) - Image height

### YOLO Node
- `model_path` (string, default: "yolov5s.pt") - Model file path
- `confidence_threshold` (float, default: 0.5) - Minimum confidence
- `nms_threshold` (float, default: 0.4) - NMS IoU threshold
- `detect_classes` (list, default: []) - Classes to detect (empty = all)

### Color Detection Node
- `target_color` (string, default: "red") - Color to detect
- `hsv_lower` (list) - Lower HSV threshold
- `hsv_upper` (list) - Upper HSV threshold
- `min_area` (int, default: 500) - Minimum detection area

### Edge Detection Node
- `algorithm` (string, default: "canny") - Algorithm (canny, sobel, laplacian, scharr)
- `canny_threshold1` (int, default: 50) - Canny lower threshold
- `canny_threshold2` (int, default: 150) - Canny upper threshold

### ArUco Node
- `dictionary_id` (string, default: "DICT_4X4_50") - ArUco dictionary
- `marker_size` (float, default: 0.05) - Marker size in meters
- `draw_axes` (bool, default: true) - Draw coordinate axes

## 🎓 Tutorials

### 1. Basic Camera Setup
```bash
# Start camera
ros2 run perception_demos usb_camera_node

# View image
ros2 run rqt_image_view rqt_image_view
```

### 2. Object Detection Pipeline
```bash
# Terminal 1: Camera
ros2 run perception_demos usb_camera_node

# Terminal 2: YOLO Detection
ros2 run perception_demos yolo_detector_node

# Terminal 3: Visualize
ros2 run rqt_image_view rqt_image_view /yolo/visualization
```

### 3. Marker-Based Localization
```bash
# Generate markers first
ros2 run perception_demos marker_generator_node

# Print markers from ./markers/ directory

# Detect markers
ros2 launch perception_demos marker_detection_launch.py

# View TF transforms
ros2 run tf2_tools view_frames
```

### 4. Depth Perception
```bash
# Launch depth pipeline
ros2 launch perception_demos depth_perception_launch.py

# Visualize point cloud in RViz2
rviz2
# Add PointCloud2 display, topic: /pointcloud
```

## 🔧 Troubleshooting

### Camera not opening
- Check camera permissions: `ls -l /dev/video*`
- Try different camera index: `-p camera_index:=1`
- Verify camera works: `cheese` or `v4l2-ctl --list-devices`

### YOLO model not found
- Download model: `wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt`
- Set model path parameter

### No detections
- Adjust confidence threshold lower
- Check camera is publishing: `ros2 topic echo /camera/image_raw`
- Verify image quality

## 📚 Dependencies

- ROS2 Humble or later
- OpenCV 4.x
- NumPy
- cv_bridge
- image_transport
- vision_msgs
- geometry_msgs
- sensor_msgs
- tf2_ros

## 🤝 Contributing

Contributions welcome! Add more perception algorithms, improve existing nodes, or create new examples.

## 📄 License

Apache 2.0

## 🔗 Resources

- [OpenCV Documentation](https://docs.opencv.org/)
- [YOLO](https://github.com/ultralytics/ultralytics)
- [ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [AprilTags](https://april.eecs.umich.edu/software/apriltag)
- [Intel RealSense](https://www.intelrealsense.com/)
- [ZED Camera](https://www.stereolabs.com/docs/)

## 🎯 Applications

This package enables:
- **Human-Robot Interaction** - Face detection, gesture recognition, pose estimation
- **Object Manipulation** - 3D object localization, grasp planning
- **Navigation** - Obstacle detection, depth mapping
- **Surveillance** - Object tracking, anomaly detection
- **AR/VR** - Marker tracking, 6DOF pose estimation
- **Quality Inspection** - Defect detection, measurement

## 📈 Performance Tips

1. **Reduce resolution** for real-time performance
2. **Use GPU** for deep learning models (set `use_gpu:=true`)
3. **Downsample point clouds** for large scenes
4. **Adjust detection thresholds** based on your scene
5. **Use appropriate codec** for video recording (H264 for compression)

---

**Total Nodes:** 30 | **Lines of Code:** ~6000+ | **Topics:** 50+

Built with ❤️ for ROS2 Perception
