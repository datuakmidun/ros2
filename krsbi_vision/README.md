# krsbi_vision

Computer vision package for KRSBI-B Soccer Robot with YOLOv8, Kalman filter tracking, and dual camera support.

## 📋 Overview

This package provides complete vision system for the robot:

- **Dual Camera Support**: Front camera + Omni fisheye 360°
- **YOLOv8 Detection**: Ball, robot, and goalpost detection
- **Kalman Filter Tracking**: Smooth tracking with physics model
- **Color-based Detection**: Fallback HSV color detection
- **Vision Fusion**: Multi-camera sensor fusion

## 📁 Package Structure

```
krsbi_vision/
├── krsbi_vision/
│   ├── __init__.py
│   ├── camera_node.py          # Front camera
│   ├── omni_camera_node.py     # Omni fisheye camera
│   ├── yolo_detector.py        # YOLOv8 detector
│   ├── ball_detector.py        # Ball detection (YOLO+color)
│   ├── ball_tracker.py         # Kalman filter tracker
│   ├── robot_detector.py       # Robot/obstacle detection
│   ├── field_detector.py       # Field and line detection
│   ├── vision_fusion.py        # Multi-camera fusion
│   ├── color_calibrator.py     # HSV calibration tool
│   ├── kalman_filter.py        # Kalman filter implementation
│   └── utils.py                # Vision utilities
├── config/
│   ├── camera_config.yaml      # Camera settings
│   ├── detection_config.yaml   # Detection parameters
│   └── tracking_config.yaml    # Kalman filter settings
├── models/
│   └── README.md               # YOLO model info
├── launch/
│   ├── vision_bringup.launch.py
│   └── calibration.launch.py
├── package.xml
├── setup.py
├── README.md
└── CHANGELOG.md
```

## 🚀 Usage

### Launch Vision System

```bash
# Build
cd ~/krsbi_ws
colcon build --packages-select krsbi_vision
source install/setup.bash

# Launch all vision nodes
ros2 launch krsbi_vision vision_bringup.launch.py

# Without YOLO (color only)
ros2 launch krsbi_vision vision_bringup.launch.py use_yolo:=false

# Without debug images
ros2 launch krsbi_vision vision_bringup.launch.py publish_debug:=false
```

### Run Individual Nodes

```bash
# Front camera only
ros2 run krsbi_vision camera_node

# Omni camera
ros2 run krsbi_vision omni_camera_node

# Ball detector
ros2 run krsbi_vision ball_detector

# YOLO detector
ros2 run krsbi_vision yolo_detector
```

### Calibration Tools

```bash
# Color calibration
ros2 launch krsbi_vision calibration.launch.py tool:=color

# Omni camera color calibration
ros2 launch krsbi_vision calibration.launch.py tool:=color camera:=omni
```

## 📊 Topics

### Published

| Topic                                | Type          | Description           |
| ------------------------------------ | ------------- | --------------------- |
| `/krsbi/camera/front/image_raw`      | Image         | Front camera image    |
| `/krsbi/camera/omni/image_raw`       | Image         | Omni camera (fisheye) |
| `/krsbi/camera/omni/image_unwrapped` | Image         | Omni panoramic view   |
| `/krsbi/vision/ball`                 | BallPosition  | Ball detection        |
| `/krsbi/vision/ball/tracked`         | PointStamped  | Tracked ball position |
| `/krsbi/vision/ball/predicted`       | PoseArray     | Predicted trajectory  |
| `/krsbi/vision/robots`               | PoseArray     | Detected robots       |
| `/krsbi/vision/world/ball`           | PointStamped  | Ball in world coords  |
| `/krsbi/vision/world/obstacles`      | OccupancyGrid | Obstacle map          |

### Debug Topics

| Topic                        | Type  | Description         |
| ---------------------------- | ----- | ------------------- |
| `/krsbi/vision/ball/debug`   | Image | Ball detection viz  |
| `/krsbi/vision/robots/debug` | Image | Robot detection viz |
| `/krsbi/vision/field/debug`  | Image | Field detection viz |

## 🔧 Configuration

### YOLOv8 Settings (detection_config.yaml)

```yaml
yolo:
  model_path: "yolov8s.pt"
  device: "cpu" # or "cuda"
  confidence_threshold: 0.5
  input_size: 416
```

### Kalman Filter (tracking_config.yaml)

```yaml
ball_tracker:
  model: "constant_acceleration"
  process_noise:
    position: 0.1
    velocity: 1.0
    acceleration: 5.0
  measurement_noise:
    position: 2.0
  physics:
    friction: 0.3
    max_velocity: 3.0
```

### Omni Camera (camera_config.yaml)

```yaml
omni_camera:
  fisheye:
    mirror_center_x: 320
    mirror_center_y: 240
    inner_radius: 60
    outer_radius: 230
    unwrap:
      output_width: 640
      output_height: 120
```

## 🎯 Detection Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Vision Pipeline                           │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐     ┌──────────────┐                      │
│  │ Front Camera │     │ Omni Camera  │                      │
│  │   640x480    │     │  360° Fisheye│                      │
│  └──────┬───────┘     └──────┬───────┘                      │
│         │                    │                               │
│         ▼                    ▼                               │
│  ┌──────────────────────────────────────────┐               │
│  │           Image Preprocessing             │               │
│  │   - Undistort, CLAHE, Fisheye Unwrap     │               │
│  └──────────────────┬───────────────────────┘               │
│                     │                                        │
│         ┌───────────┼───────────┐                           │
│         ▼           ▼           ▼                           │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐                     │
│  │  YOLOv8  │ │  Color   │ │  Field   │                     │
│  │ Detector │ │ Detector │ │ Detector │                     │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘                     │
│       │            │            │                            │
│       └──────┬─────┘            │                            │
│              ▼                  │                            │
│       ┌──────────────┐          │                            │
│       │   Kalman     │          │                            │
│       │   Filter     │          │                            │
│       └──────┬───────┘          │                            │
│              │                  │                            │
│              ▼                  ▼                            │
│       ┌────────────────────────────────┐                    │
│       │       Vision Fusion            │                    │
│       │  - World coordinate transform  │                    │
│       │  - Multi-camera fusion         │                    │
│       │  - Obstacle grid               │                    │
│       └────────────────────────────────┘                    │
└─────────────────────────────────────────────────────────────┘
```

## 📦 Dependencies

### Python Packages

```bash
pip install ultralytics opencv-python numpy scipy filterpy
```

### ROS 2 Packages

- `cv_bridge`
- `image_transport`
- `sensor_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `nav_msgs`

## 🎮 Kalman Filter

### Ball Tracker Features

- **Constant Acceleration** model for ball physics
- **Friction simulation** for realistic prediction
- **Trajectory prediction** up to 1 second ahead
- **Confidence decay** when ball is lost
- **Track management** with confirmation/deletion thresholds

### Multi-Object Tracker Features

- **Hungarian algorithm** for optimal assignment
- **Mahalanobis distance** for gating
- **Track ID management** for persistent tracking

## 📐 Coordinate Systems

```
Robot Frame (base_link):
     Y (forward)
     ^
     |
     |
     +-----> X (right)
```

```
Image Frame:
     +-----> U
     |
     |
     V
```

## 🐛 Troubleshooting

### Camera Not Found

```bash
# List cameras
v4l2-ctl --list-devices

# Test camera
ffplay /dev/video0
```

### YOLO Slow Performance

```bash
# Use smaller model
ros2 run krsbi_vision yolo_detector --ros-args -p model_path:=yolov8n.pt

# Use CUDA (if NVIDIA GPU)
ros2 run krsbi_vision yolo_detector --ros-args -p device:=cuda
```

### Color Detection Issues

```bash
# Run calibration tool
ros2 launch krsbi_vision calibration.launch.py tool:=color

# Adjust HSV thresholds interactively
```

## 📝 License

MIT License

## 👥 Authors

- KRSBI-B Team
