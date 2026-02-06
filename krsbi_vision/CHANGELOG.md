# Changelog

All notable changes to the `krsbi_vision` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-06

### Added

#### Camera Nodes

- **camera_node.py** - Front camera capture:
  - Threading-based frame capture
  - CameraInfo publishing
  - Configurable resolution and FPS

- **omni_camera_node.py** - Omnidirectional fisheye camera:
  - Real-time fisheye unwrapping to panoramic view
  - Precomputed remap tables for fast processing
  - Configurable mirror center and radii

#### Detection Nodes

- **yolo_detector.py** - YOLOv8 detection:
  - Multi-camera input support
  - Custom model loading
  - Configurable confidence thresholds
  - Debug visualization

- **ball_detector.py** - Ball detection:
  - YOLO + color-based detection fusion
  - Kalman filter integration
  - Distance estimation
  - Multi-camera support

- **robot_detector.py** - Robot/obstacle detection:
  - Multi-object tracking
  - Team color detection (blue/yellow)
  - Persistent track IDs

- **field_detector.py** - Field detection:
  - Green field segmentation
  - White line detection (Hough)
  - Field boundary detection

#### Tracking

- **kalman_filter.py** - Kalman filter implementation:
  - Constant Velocity (CV) model
  - Constant Acceleration (CA) model
  - Physics-based ball tracking (friction)
  - Trajectory prediction
  - Multi-object tracker with Hungarian algorithm

- **ball_tracker.py** - Standalone ball tracker:
  - Subscribes to detections
  - Publishes tracked/predicted positions
  - Velocity estimation

#### Fusion

- **vision_fusion.py** - Multi-camera fusion:
  - Pixel to world coordinate transformation
  - Camera sensor fusion
  - Obstacle occupancy grid
  - TF broadcasting

#### Tools

- **color_calibrator.py** - HSV calibration:
  - Interactive OpenCV trackbars
  - Preset management (ball, field, line)
  - Save/load configuration
  - Live visualization

#### Utilities

- **utils.py** - Vision utilities:
  - Fisheye unwrapping functions
  - Coordinate transformations
  - Image preprocessing (CLAHE, blur)
  - HSV masking
  - Drawing utilities

#### Configuration

- **camera_config.yaml** - Camera parameters:
  - Front and omni camera settings
  - Fisheye unwrap configuration
  - Calibration intrinsics

- **detection_config.yaml** - Detection settings:
  - YOLOv8 parameters
  - Color thresholds (HSV)
  - Size filtering

- **tracking_config.yaml** - Tracking settings:
  - Kalman filter noise parameters
  - Track management thresholds
  - Sensor fusion weights

#### Launch

- **vision_bringup.launch.py** - Full vision system
- **calibration.launch.py** - Calibration tools

### Camera Setup

| Camera | Resolution | FPS | Purpose              |
| ------ | ---------- | --- | -------------------- |
| Front  | 640x480    | 30  | Ball, goal detection |
| Omni   | 640x480    | 30  | 360° awareness       |

### Detection Performance

| Method   | Target | Accuracy | Latency |
| -------- | ------ | -------- | ------- |
| YOLOv8s  | Ball   | ~85%     | ~65ms   |
| Color    | Ball   | ~70%     | ~5ms    |
| Combined | Ball   | ~90%     | ~70ms   |

---

## [Unreleased]

### Planned

- Camera intrinsic calibration tool
- Deep learning field segmentation
- Goal detection node
- Pose estimation for robots
- GPU acceleration (TensorRT)
- Recording and replay functionality

---

## Version History

| Version | Date       | Description                                 |
| ------- | ---------- | ------------------------------------------- |
| 1.0.0   | 2026-02-06 | Initial release with dual camera and Kalman |
