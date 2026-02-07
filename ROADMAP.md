# 🤖 KRSBI-B Soccer Robot - Development Roadmap

> **Project:** KRSBI-B Soccer Robot  
> **Platform:** ROS 2 Jazzy | Intel NUC + Arduino Mega  
> **Communication:** Serial USB TTL  
> **Language:** Python

---

## 🔧 Hardware Specifications

### Vision System

| Component        | Description                         |
| ---------------- | ----------------------------------- |
| **Kamera Omni**  | Fisheye 360° omnidirectional camera |
| **Kamera Front** | Logitech webcam (forward-facing)    |

### Actuators (via Arduino Mega)

| Component     | Driver   | Description                                    |
| ------------- | -------- | ---------------------------------------------- |
| **Motor 1-3** | BTS7960  | 3x PG45 DC Motor dengan encoder (3-wheel omni) |
| **Gripper**   | BTS7960  | Motor gripper untuk mencengkram bola           |
| **Kicker**    | 2x Relay | Capacitor charging + discharge ke solenoid     |

### Sensors (via Arduino Mega)

| Component    | Description                       |
| ------------ | --------------------------------- |
| **Encoder**  | 3x Encoder pada motor PG45        |
| **IMU**      | Sensor orientasi (MPU6050/BNO055) |
| **Sharp GP** | Sensor jarak infrared             |

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Package Dependencies](#package-dependencies)
3. [Development Phases](#development-phases)
4. [Package Roadmaps](#package-roadmaps)
   - [krsbi_msgs](#1-krsbi_msgs)
   - [krsbi_description](#2-krsbi_description)
   - [krsbi_interface](#3-krsbi_interface)
   - [krsbi_comm](#4-krsbi_comm)
   - [krsbi_vision](#5-krsbi_vision)
   - [krsbi_control](#6-krsbi_control)
   - [krsbi_decision](#7-krsbi_decision)
5. [Integration Timeline](#integration-timeline)
6. [Testing Strategy](#testing-strategy)

---

## Overview

Roadmap ini mendefinisikan tahapan pengembangan setiap package ROS 2 untuk robot sepak bola KRSBI-B. Pengembangan dilakukan secara bertahap dengan prioritas pada package yang menjadi dependensi package lainnya.

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Intel NUC                                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ krsbi_vision │  │krsbi_decision│  │    krsbi_control     │   │
│  │  (Camera)    │──│  (Strategy)  │──│  (Motion Planning)   │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
│         │                 │                     │                │
│         └─────────────────┼─────────────────────┘                │
│                           │                                      │
│                    ┌──────────────┐                              │
│                    │  krsbi_comm  │                              │
│                    │(micro-ROS)   │                              │
│                    └──────┬───────┘                              │
└───────────────────────────┼─────────────────────────────────────┘
                            │ Serial
                    ┌───────┴───────┐
                    │  Arduino Mega │
                    │   (Actuator)  │
                    └───────────────┘
```

---

## Package Dependencies

```
krsbi_msgs (Foundational - No Dependencies)
    │
    ├── krsbi_description (Robot Model)
    │
    ├── krsbi_interface (Service/Action Definitions)
    │       │
    │       └── krsbi_comm (Hardware Communication)
    │               │
    │               ├── krsbi_control (Motion Control)
    │               │
    │               └── krsbi_vision (Perception)
    │                       │
    │                       └── krsbi_decision (AI/Strategy)
    └──────────────────────────────────────┘
```

### Urutan Pengembangan (Priority Order)

1. **krsbi_msgs** - Message definitions (HARUS PERTAMA)
2. **krsbi_description** - Robot model/URDF
3. **krsbi_interface** - Service & Action definitions
4. **krsbi_comm** - Serial communication
5. **krsbi_vision** - Computer vision
6. **krsbi_control** - Motion control
7. **krsbi_decision** - Game strategy AI

---

## Development Phases

| Phase      | Nama          | Durasi   | Deskripsi                       |
| ---------- | ------------- | -------- | ------------------------------- |
| 🔵 Phase 1 | Foundation    | 2 minggu | Message, Description, Interface |
| 🟢 Phase 2 | Communication | 2 minggu | Serial & micro-ROS setup        |
| 🟡 Phase 3 | Perception    | 3 minggu | Vision system                   |
| 🟠 Phase 4 | Control       | 3 minggu | Motion control                  |
| 🔴 Phase 5 | Intelligence  | 3 minggu | Decision making                 |
| 🟣 Phase 6 | Integration   | 2 minggu | Full system testing             |

---

## Package Roadmaps

---

### 1. krsbi_msgs

> **Tujuan:** Mendefinisikan custom messages untuk komunikasi antar-node  
> **Priority:** 🔴 CRITICAL (Foundation Package)  
> **Status:** ✅ COMPLETED  
> **Estimated Duration:** 1 minggu

#### 1.1 Structure

```
krsbi_msgs/
├── msg/
│   ├── BallPosition.msg      # Posisi bola terdeteksi
│   ├── RobotState.msg        # Status keseluruhan robot
│   ├── MotorCommand.msg      # Perintah ke motor (3 motor)
│   ├── MotorFeedback.msg     # Feedback dari encoder
│   ├── SensorData.msg        # Gabungan semua sensor
│   ├── ImuData.msg           # Data IMU
│   ├── DistanceSensors.msg   # Data sensor jarak Sharp GP
│   ├── GripperState.msg      # Status gripper
│   ├── KickerState.msg       # Status kicker/penendang
│   ├── GameState.msg         # Status permainan
│   ├── DetectedObject.msg    # Objek terdeteksi
│   ├── DetectedObjects.msg   # Kumpulan objek terdeteksi
│   ├── FieldLines.msg        # Garis lapangan
│   ├── RobotPose.msg         # Pose robot di lapangan
│   └── TeamRobots.msg        # Info tim robot
├── srv/
│   ├── SetMotorSpeed.srv     # Set kecepatan motor
│   ├── SetGripper.srv        # Kontrol gripper
│   ├── ChargeKicker.srv      # Charge capacitor kicker
│   ├── Kick.srv              # Eksekusi tendangan
│   ├── GetRobotState.srv     # Get status robot
│   ├── Calibrate.srv         # Kalibrasi sensor
│   └── SetGameState.srv      # Set game state
├── action/
│   ├── MoveTo.action         # Gerak ke posisi
│   ├── RotateTo.action       # Putar ke heading
│   ├── KickBall.action       # Tendang bola (full sequence)
│   └── GrabBall.action       # Ambil bola dengan gripper
├── CMakeLists.txt
├── package.xml
└── README.md
```

#### 1.2 Development Tasks

| Task   | Deskripsi                                               | Status  |
| ------ | ------------------------------------------------------- | ------- |
| 1.1.1  | Setup package untuk CMake msg generation                | ✅ DONE |
| 1.1.2  | Definisi `BallPosition.msg` (x, y, distance, camera)    | ✅ DONE |
| 1.1.3  | Definisi `RobotState.msg` (pose, velocity, battery)     | ✅ DONE |
| 1.1.4  | Definisi `MotorCommand.msg` (3 motor, velocity/RPM)     | ✅ DONE |
| 1.1.5  | Definisi `MotorFeedback.msg` (encoder, RPM)             | ✅ DONE |
| 1.1.6  | Definisi `SensorData.msg` (composite sensor data)       | ✅ DONE |
| 1.1.7  | Definisi `ImuData.msg` (orientation, gyro, accel)       | ✅ DONE |
| 1.1.8  | Definisi `DistanceSensors.msg` (Sharp GP sensors)       | ✅ DONE |
| 1.1.9  | Definisi `GripperState.msg` (position, ball detect)     | ✅ DONE |
| 1.1.10 | Definisi `KickerState.msg` (capacitor, relay, safety)   | ✅ DONE |
| 1.1.11 | Definisi `GameState.msg` (phase, score, set play)       | ✅ DONE |
| 1.1.12 | Definisi `DetectedObject.msg` (ball, robot, goal)       | ✅ DONE |
| 1.1.13 | Definisi `DetectedObjects.msg` (collection)             | ✅ DONE |
| 1.1.14 | Definisi `FieldLines.msg` (lines, corners)              | ✅ DONE |
| 1.1.15 | Definisi `RobotPose.msg` (field coordinates)            | ✅ DONE |
| 1.1.16 | Definisi `TeamRobots.msg` (team coordination)           | ✅ DONE |
| 1.1.17 | Definisi services (motor, gripper, kicker, calibrate)   | ✅ DONE |
| 1.1.18 | Definisi actions (MoveTo, RotateTo, KickBall, GrabBall) | ✅ DONE |
| 1.1.19 | Build & test msg generation                             | ⬜ TODO |

#### 1.3 Key Message Specifications

```
# MotorCommand.msg (3-wheel omni)
# Motor arrangement (top view):
#        FRONT (0°)
#           M1
#          /  \
#         /    \
#        M2----M3

float64 motor1_rpm                 # Motor 1 (front)
float64 motor2_rpm                 # Motor 2 (rear left)
float64 motor3_rpm                 # Motor 3 (rear right)
float64 linear_x                   # Velocity mode: m/s
float64 linear_y                   # Velocity mode: m/s
float64 angular_z                  # Velocity mode: rad/s
uint8 mode                         # MODE_VELOCITY=0, MODE_RPM=1
bool enable
bool emergency_stop

# BallPosition.msg
float64 x, y, z                    # Position (meters)
float64 distance                   # Polar distance
float64 angle                      # Polar angle
float64 confidence                 # 0.0 - 1.0
bool is_visible
uint8 camera_source                # CAMERA_OMNI=0, CAMERA_FRONT=1, CAMERA_FUSED=2

# KickerState.msg (capacitor + 2 relay)
float32 voltage                    # Capacitor voltage
float32 charge_percentage          # 0-100%
bool relay_charge                  # Charging relay
bool relay_kick                    # Discharge relay
uint8 state                        # IDLE, CHARGING, READY, KICKING, COOLDOWN, ERROR
uint8 power_level                  # LOW, MEDIUM, HIGH
bool ready_to_kick                 # All safety checks passed
```

#### 1.4 Acceptance Criteria

- [x] Package structure dengan CMakeLists.txt
- [x] 15 message definitions
- [x] 7 service definitions
- [x] 4 action definitions
- [x] Dokumentasi lengkap (README.md)
- [ ] Build & test msg generation

---

### 2. krsbi_description

> **Tujuan:** Definisi model robot (URDF) untuk visualisasi dan simulasi  
> **Priority:** 🟡 MEDIUM  
> **Status:** ✅ COMPLETED  
> **Estimated Duration:** 1 minggu

#### 2.1 Structure

```
krsbi_description/
├── krsbi_description/
│   ├── __init__.py
│   └── state_publisher.py      # Custom joint state publisher
├── urdf/
│   ├── robot.urdf.xacro        # Main robot description
│   ├── materials.xacro         # Color definitions
│   ├── properties.xacro        # Dimensions & inertia
│   ├── wheel.xacro             # Omni wheel macro
│   ├── sensors.xacro           # Cameras, IMU, distance sensors
│   └── actuators.xacro         # Gripper and kicker
├── config/
│   ├── robot_params.yaml       # Robot parameters
│   └── joint_limits.yaml       # Joint configuration
├── launch/
│   ├── display.launch.py
│   ├── robot_state_publisher.launch.py
│   └── view_frames.launch.py
├── rviz/
│   └── display.rviz
├── meshes/                     # For CAD exports (placeholder)
├── package.xml
├── setup.py
├── README.md
└── CHANGELOG.md
```

#### 2.2 Development Tasks

| Task  | Deskripsi                                    | Status  |
| ----- | -------------------------------------------- | ------- |
| 2.2.1 | Buat base URDF dengan dimensi robot          | ✅ DONE |
| 2.2.2 | Definisi wheels (3 omni-wheels)              | ✅ DONE |
| 2.2.3 | Tambahkan camera mount dan sensor positions  | ✅ DONE |
| 2.2.4 | Definisi collision geometry                  | ✅ DONE |
| 2.2.5 | Buat launch file untuk robot_state_publisher | ✅ DONE |
| 2.2.6 | Setup RViz config untuk visualisasi          | ✅ DONE |
| 2.2.7 | Export meshes dari CAD (jika tersedia)       | ⬜ TODO |
| 2.2.8 | Validasi URDF dengan `check_urdf`            | ⬜ TODO |

#### 2.3 Robot Specifications

```yaml
# config/robot_specs.yaml
robot:
  name: "krsbi_robot"
  base:
    shape: "cylindrical" # or "triangular"
    diameter: 0.40 # meters
    height: 0.35

  # 3-wheel omni configuration
  #        FRONT (0°)
  #           M1
  #          /  \
  #         /    \
  #        M2----M3
  wheels:
    type: "omni_3"
    count: 3
    diameter: 0.10
    positions: # [x, y, z] from center, angle from front
      - name: "front"
        position: [0.17, 0.0, 0.05]
        angle: 0 # degrees from front
      - name: "rear_left"
        position: [-0.085, 0.147, 0.05]
        angle: 120
      - name: "rear_right"
        position: [-0.085, -0.147, 0.05]
        angle: 240

  cameras:
    omni:
      type: "fisheye_360"
      position: [0, 0, 0.35] # center top
      fov: 360
    front:
      type: "logitech_webcam"
      position: [0.18, 0, 0.25]
      orientation: [0, 0.2, 0] # pitch down slightly
      fov: 78

  sensors:
    imu:
      position: [0, 0, 0.15]
    sharp_gp: # distance sensors
      positions:
        front: [0.20, 0, 0.10]
        left: [0, 0.20, 0.10]
        right: [0, -0.20, 0.10]

  actuators:
    gripper:
      position: [0.20, 0, 0.08]
    kicker:
      position: [0.18, 0, 0.05]
```

#### 2.4 Acceptance Criteria

- [ ] URDF dapat di-load tanpa error
- [ ] Visualisasi di RViz berjalan dengan benar
- [ ] TF tree complete dan valid
- [ ] Dimensi sesuai dengan robot fisik

---

### 3. krsbi_interface

> **Tujuan:** Definisi interface level tinggi (parameters, configs, launch)  
> **Priority:** 🟡 MEDIUM  
> **Status:** ✅ COMPLETED  
> **Estimated Duration:** 1 minggu

#### 3.1 Structure

```
krsbi_interface/
├── krsbi_interface/
│   ├── __init__.py
│   ├── constants.py           # Game states, roles, limits, topics
│   ├── utils.py               # Utility functions
│   ├── system_monitor.py      # System health monitoring node
│   └── param_server.py        # Parameter server node
├── config/
│   ├── robot_params.yaml      # Robot identity, hardware, limits
│   ├── vision_params.yaml     # Camera and detection settings
│   ├── control_params.yaml    # Motion control, behaviors
│   └── game_params.yaml       # Field dimensions, game rules
├── launch/
│   ├── robot_bringup.launch.py
│   ├── match.launch.py
│   └── simulation.launch.py
├── package.xml
├── setup.py
├── README.md
└── CHANGELOG.md
```

#### 3.2 Development Tasks

| Task  | Deskripsi                                         | Status  |
| ----- | ------------------------------------------------- | ------- |
| 3.2.1 | Definisi parameter files untuk setiap subsystem   | ✅ DONE |
| 3.2.2 | Buat constants.py dengan game rules & constraints | ✅ DONE |
| 3.2.3 | Buat robot_bringup.launch.py                      | ✅ DONE |
| 3.2.4 | Buat match.launch.py untuk mode pertandingan      | ✅ DONE |
| 3.2.5 | Implementasi system_monitor node                  | ✅ DONE |
| 3.2.6 | Buat utility functions                            | ✅ DONE |
| 3.2.7 | Dokumentasi lengkap                               | ✅ DONE |

#### 3.3 Parameter Specifications

```yaml
# config/robot_params.yaml
robot:
  robot_id: 1
  team_color: "blue" # or "yellow"
  role: "striker" # striker, goalkeeper, defender

hardware:
  serial_port: "/dev/ttyUSB0"
  baudrate: 115200

limits:
  max_linear_velocity: 1.5 # m/s
  max_angular_velocity: 3.14 # rad/s
  max_acceleration: 2.0 # m/s²
```

```yaml
# config/game_params.yaml
field:
  length: 9.0 # meters
  width: 6.0
  goal_width: 2.6
  penalty_area_length: 1.0
  penalty_area_width: 3.0
  center_circle_radius: 0.75

game:
  half_duration: 600 # seconds (10 minutes)
  max_robots: 4
```

#### 3.4 Acceptance Criteria

- [ ] Semua launch files berjalan tanpa error
- [ ] Parameter dapat di-load dan diakses node
- [ ] System monitor berjalan dan publish diagnostics
- [ ] Dokumentasi parameter lengkap

---

### 4. krsbi_comm

> **Tujuan:** Komunikasi serial antara Intel NUC dan Arduino Mega via micro-ROS  
> **Priority:** 🔴 CRITICAL  
> **Status:** ✅ COMPLETED  
> **Estimated Duration:** 2 minggu

#### 4.1 Structure

```
krsbi_comm/
├── krsbi_comm/
│   ├── __init__.py
│   ├── serial_node.py         # Main ROS 2 node
│   ├── protocol.py            # Packet format and commands
│   ├── crc_utils.py           # CRC-8 checksum
│   └── protocol_test.py       # Protocol testing
├── config/
│   ├── serial_config.yaml     # Serial port settings
│   └── protocol_config.yaml   # Protocol specification
├── launch/
│   └── comm_bringup.launch.py
├── package.xml
├── setup.py
├── README.md
└── CHANGELOG.md
```

#### 4.2 Development Tasks

| Task   | Deskripsi                                       | Status  |
| ------ | ----------------------------------------------- | ------- |
| 4.2.1  | Definisi communication protocol (packet format) | ✅ DONE |
| 4.2.2  | Implementasi CRC checksum untuk data integrity  | ✅ DONE |
| 4.2.3  | Buat serial_node.py untuk koneksi dasar         | ✅ DONE |
| 4.2.4  | Implementasi receive handler untuk sensor data  | ✅ DONE |
| 4.2.5  | Implementasi send handler untuk motor commands  | ✅ DONE |
| 4.2.6  | Buat reconnection & error handling              | ✅ DONE |
| 4.2.7  | Implementasi heartbeat mechanism                | ✅ DONE |
| 4.2.8  | Buat protocol test node                         | ✅ DONE |
| 4.2.9  | Setup micro-ROS agent configuration             | ⬜ TODO |
| 4.2.10 | Integration test dengan Arduino                 | ⬜ TODO |

#### 4.3 Protocol Specification

```python
# protocol.py
"""
Packet Format:
+--------+--------+--------+--------+--------+...+--------+--------+
| START  |  LEN   |  CMD   | DATA_0 | DATA_1 |...| DATA_N |  CRC   |
+--------+--------+--------+--------+--------+...+--------+--------+
|  0xAA  | 1 byte | 1 byte |      N bytes         | 1 byte |
+--------+--------+--------+------------------------+--------+

Commands (CMD):
  0x01 - Motor velocity command
  0x02 - Kicker command
  0x03 - Request sensor data
  0x10 - Sensor data response
  0x11 - Status response
  0xFF - Emergency stop
"""

from dataclasses import dataclass
from enum import IntEnum

class Command(IntEnum):
    MOTOR_VELOCITY = 0x01
    KICKER = 0x02
    REQUEST_SENSORS = 0x03
    SENSOR_RESPONSE = 0x10
    STATUS_RESPONSE = 0x11
    EMERGENCY_STOP = 0xFF

@dataclass
class Packet:
    command: Command
    data: bytes

    START_BYTE = 0xAA
```

#### 4.4 Node Implementation

```python
# serial_node.py (skeleton)
import rclpy
from rclpy.node import Node
import serial
from krsbi_msgs.msg import MotorCommand, SensorData

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # Publishers
        self.sensor_pub = self.create_publisher(
            SensorData, 'sensor_data', 10)

        # Subscribers
        self.motor_sub = self.create_subscription(
            MotorCommand, 'motor_command',
            self.motor_callback, 10)

        # Serial connection
        self.serial = None
        self.connect()

        # Timer for reading
        self.timer = self.create_timer(0.01, self.read_loop)
```

#### 4.5 Acceptance Criteria

- [ ] Koneksi serial stabil tanpa packet loss
- [ ] Latency < 10ms untuk command-response
- [ ] Auto-reconnect saat connection lost
- [ ] CRC validation berjalan dengan benar
- [ ] Unit tests passing

---

### 5. krsbi_vision

> **Tujuan:** Computer vision untuk deteksi bola, garis lapangan, dan obstacles  
> **Priority:** 🔴 CRITICAL  
> **Status:** ✅ COMPLETED  
> **Estimated Duration:** 3 minggu

#### 5.1 Structure

```
krsbi_vision/
├── krsbi_vision/
│   ├── __init__.py
│   ├── camera_node.py          # Front camera
│   ├── omni_camera_node.py     # Omni fisheye 360°
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

#### 5.2 Development Tasks

| Task                   | Deskripsi                            | Status  |
| ---------------------- | ------------------------------------ | ------- |
| **Camera Setup**       |                                      |         |
| 5.2.1                  | Setup camera driver (usb_cam / v4l2) | ✅ DONE |
| 5.2.2                  | Kalibrasi intrinsic camera           | ✅ DONE |
| 5.2.3                  | Implementasi camera_node.py          | ✅ DONE |
| 5.2.4                  | Implementasi omni_camera_node.py     | ✅ DONE |
| **Ball Detection**     |                                      |         |
| 5.2.5                  | Color-based ball detection (HSV)     | ✅ DONE |
| 5.2.6                  | ML-based ball detection (YOLOv8)     | ✅ DONE |
| 5.2.7                  | Ball tracking dengan Kalman Filter   | ✅ DONE |
| 5.2.8                  | Ball position estimation (3D)        | ✅ DONE |
| **Field Detection**    |                                      |         |
| 5.2.9                  | Field line detection                 | ✅ DONE |
| 5.2.10                 | Field boundary detection             | ✅ DONE |
| 5.2.11                 | Localization assist dari field lines | ⬜ TODO |
| **Goal Detection**     |                                      |         |
| 5.2.12                 | Goal post detection                  | ⬜ TODO |
| 5.2.13                 | Goal direction estimation            | ⬜ TODO |
| **Obstacle Detection** |                                      |         |
| 5.2.14                 | Robot/obstacle detection             | ✅ DONE |
| 5.2.15                 | Obstacle position mapping            | ✅ DONE |
| **Calibration**        |                                      |         |
| 5.2.16                 | Color calibration tool (GUI)         | ✅ DONE |
| 5.2.17                 | Save/load calibration data           | ✅ DONE |
| **Advanced**           |                                      |         |
| 5.2.18                 | Vision fusion (multi-camera)         | ✅ DONE |
| 5.2.19                 | Trajectory prediction                | ✅ DONE |

#### 5.3 Node Implementation

```python
# ball_detector.py (skeleton)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from krsbi_msgs.msg import BallPosition

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # Parameters
        self.declare_parameter('hsv_lower', [0, 100, 100])
        self.declare_parameter('hsv_upper', [10, 255, 255])

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw',
            self.image_callback, 10)

        # Publishers
        self.ball_pub = self.create_publisher(
            BallPosition, 'ball_position', 10)
        self.debug_pub = self.create_publisher(
            Image, 'ball_detection/debug', 10)

    def detect_ball(self, frame):
        """Detect ball using color thresholding + Hough circles"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Morphological operations
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)
            return (x, y, radius)
        return None
```

#### 5.4 Color Calibration Config

```yaml
# config/color_thresholds.yaml
ball:
  color: "orange"
  hsv_lower: [5, 100, 100]
  hsv_upper: [15, 255, 255]
  min_area: 100
  max_area: 50000

field:
  color: "green"
  hsv_lower: [35, 50, 50]
  hsv_upper: [85, 255, 255]

goal:
  color: "white"
  hsv_lower: [0, 0, 200]
  hsv_upper: [180, 30, 255]
```

#### 5.5 Acceptance Criteria

- [ ] Ball detection rate > 90% pada kondisi lapangan standar
- [ ] Detection latency < 33ms (30 FPS)
- [ ] Akurasi posisi bola < 10cm pada jarak 2m
- [ ] Robust terhadap variasi pencahayaan
- [ ] Calibration tool functional

---

### 6. krsbi_control

> **Tujuan:** Motion control, path planning, dan kinematika robot  
> **Priority:** 🔴 CRITICAL  
> **Status:** ⚠️ IN PROGRESS  
> **Estimated Duration:** 3 minggu

#### 6.1 Structure

```
krsbi_control/
├── krsbi_control/
│   ├── __init__.py
│   ├── motion_controller.py     # Velocity ramping & limits
│   ├── omni_kinematics.py       # 3-wheel Omni Kinematics
│   ├── pid_controller.py        # PID implementation
│   ├── path_planner.py          # Simple P2P Planner
│   ├── trajectory_tracker.py    # Placeholder
│   ├── localization.py          # Odometry & IMU fusion
│   ├── behavior_node.py         # Behavior dispatcher
│   └── behaviors/
│       ├── __init__.py          # (Empty)
│       └── (Implemented in behavior_node per roadmap plan)
├── config/
│   └── control_config.yaml      # Consolidated config
├── launch/
│   └── control_bringup.launch.py
├── package.xml
├── setup.py
└── README.md
```

#### 6.2 Development Tasks

| Task               | Deskripsi                                      | Status  |
| ------------------ | ---------------------------------------------- | ------- |
| **Kinematics**     |                                                |         |
| 6.2.1              | Implementasi forward kinematics (3 omni-wheel) | ✅ DONE |
| 6.2.2              | Implementasi inverse kinematics                | ✅ DONE |
| 6.2.3              | Unit test kinematics                           | ⬜ TODO |
| **Motion Control** |                                                |         |
| 6.2.4              | Implementasi PID controller                    | ✅ DONE |
| 6.2.5              | Velocity ramping (acceleration limit)          | ✅ DONE |
| 6.2.6              | Implementasi motion_controller node            | ✅ DONE |
| **Path Planning**  |                                                |         |
| 6.2.7              | Simple path planner (straight line)            | ✅ DONE |
| 6.2.8              | Obstacle avoidance (local)                     | ✅ DONE |
| 6.2.9              | Trajectory tracking                            | ⬜ TODO |
| **Localization**   |                                                |         |
| 6.2.10             | Odometry dari encoder                          | ✅ DONE |
| 6.2.11             | IMU fusion                                     | ✅ DONE |
| 6.2.12             | Field-based localization correction            | ⬜ TODO |
| **Behaviors**      |                                                |         |
| 6.2.13             | GoToPosition behavior                          | ✅ DONE |
| 6.2.14             | FollowBall behavior                            | ✅ DONE |
| 6.2.15             | Dribble behavior                               | ⬜ TODO |
| 6.2.16             | Kick behavior                                  | ⬜ TODO |

#### 6.3 Omni-Wheel Kinematics

```python
# omni_kinematics.py
import numpy as np

class OmniKinematics3:
    """
    3-wheel Omni-directional robot kinematics

    Wheel arrangement (top view):
           FRONT (0°)
              M1
             /  \
            /    \
           M2----M3

    M1: Front wheel at 0° (pointing backward for forward motion)
    M2: Rear-left wheel at 120°
    M3: Rear-right wheel at 240°
    """

    def __init__(self, wheel_radius: float, robot_radius: float):
        self.R = wheel_radius
        self.L = robot_radius  # distance from center to wheel

        # Wheel angles from front (radians)
        # Each wheel's rolling direction is perpendicular to its position
        self.wheel_angles = np.array([
            np.deg2rad(0),    # M1: front
            np.deg2rad(120),  # M2: rear-left
            np.deg2rad(240),  # M3: rear-right
        ])

        # Inverse kinematics matrix
        # [w1, w2, w3]^T = (1/R) * H * [vx, vy, omega]^T
        # For wheel at angle θ, contribution is: -sin(θ)*vx + cos(θ)*vy + L*omega
        self.H_inv = np.array([
            [-np.sin(self.wheel_angles[0]), np.cos(self.wheel_angles[0]), self.L],
            [-np.sin(self.wheel_angles[1]), np.cos(self.wheel_angles[1]), self.L],
            [-np.sin(self.wheel_angles[2]), np.cos(self.wheel_angles[2]), self.L],
        ])

    def inverse(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """
        Calculate wheel velocities from robot velocity (Inverse Kinematics)

        Args:
            vx: Linear velocity x (m/s), positive = forward
            vy: Linear velocity y (m/s), positive = left
            omega: Angular velocity (rad/s), positive = CCW

        Returns:
            Array of 3 wheel velocities (rad/s)
        """
        vel = np.array([vx, vy, omega])
        wheel_vel = (1 / self.R) * self.H_inv @ vel
        return wheel_vel

    def forward(self, wheel_velocities: np.ndarray) -> tuple:
        """
        Calculate robot velocity from wheel velocities (Forward Kinematics)

        Args:
            wheel_velocities: Array of 3 wheel velocities (rad/s)

        Returns:
            Tuple of (vx, vy, omega)
        """
        H = np.linalg.pinv(self.H_inv)
        robot_vel = self.R * H @ wheel_velocities
        return tuple(robot_vel)

    def velocity_to_rpm(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """Convert velocity command to motor RPM"""
        wheel_rad_s = self.inverse(vx, vy, omega)
        wheel_rpm = wheel_rad_s * 60 / (2 * np.pi)
        return wheel_rpm
```

#### 6.4 PID Controller

```python
# pid_controller.py
from dataclasses import dataclass
import time

@dataclass
class PIDGains:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0

class PIDController:
    def __init__(self, gains: PIDGains, output_limits: tuple = None):
        self.gains = gains
        self.output_limits = output_limits
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, setpoint: float, measurement: float) -> float:
        current_time = time.time()
        error = setpoint - measurement

        if self.prev_time is None:
            dt = 0.01
        else:
            dt = current_time - self.prev_time

        # P term
        p_term = self.gains.kp * error

        # I term with anti-windup
        self.integral += error * dt
        i_term = self.gains.ki * self.integral

        # D term
        d_term = self.gains.kd * (error - self.prev_error) / dt if dt > 0 else 0

        output = p_term + i_term + d_term

        # Apply limits
        if self.output_limits:
            output = max(self.output_limits[0],
                        min(self.output_limits[1], output))

        self.prev_error = error
        self.prev_time = current_time

        return output
```

#### 6.5 Acceptance Criteria

- [ ] Kinematics sesuai dengan konfigurasi fisik robot
- [ ] PID tuned untuk response yang stabil
- [ ] Position accuracy < 5cm
- [ ] Heading accuracy < 5°
- [ ] Smooth velocity profiles

---

### 7. krsbi_decision

> **Tujuan:** Game strategy, behavior tree, dan decision making  
> **Priority:** 🟠 HIGH  
> **Status:** ✅ COMPLETED (Core)  
> **Estimated Duration:** 3 minggu

#### 7.1 Structure

```
krsbi_decision/
├── krsbi_decision/
│   ├── __init__.py
│   ├── game_controller.py       # ✅ Referee Interface
│   ├── strategy_manager.py      # ✅ Main BT Executor
│   ├── world_model/
│   │   ├── __init__.py
│   │   └── world_state.py       # ✅ Data structures
│   └── behavior_tree/
│       ├── __init__.py
│       ├── actions.py           # ✅ Leaf nodes
│       ├── conditions.py        # ✅ Leaf nodes
│       └── roles.py             # ✅ Strategy Trees
├── config/
│   └── strategy_params.yaml     # ✅ Game Rules
├── launch/
│   └── decision_bringup.launch.py
├── package.xml
├── setup.py
└── README.md
```

#### 7.2 Development Tasks

| Task                | Deskripsi                                       | Status  |
| ------------------- | ----------------------------------------------- | ------- |
| **World Model**     |                                                 |         |
| 7.2.1               | Implementasi WorldState class                   | ✅ DONE |
| 7.2.2               | Ball tracking & prediction                      | ✅ DONE |
| 7.2.3               | Robot pose estimation                           | ✅ DONE |
| 7.2.4               | Opponent tracking                               | ⬜ TODO |
| **Game Controller** |                                                 |         |
| 7.2.5               | Game state machine (Ready, Play, Pause, etc.)   | ✅ DONE |
| 7.2.6               | GameController protocol integration             | ⬜ TODO |
| 7.2.7               | Penalty/Freekick handling                       | ⬜ TODO |
| **Role Management** |                                                 |         |
| 7.2.8               | Dynamic role assignment                         | ⬜ TODO |
| 7.2.9               | Role switching logic                            | ⬜ TODO |
| **Behavior Trees**  |                                                 |         |
| 7.2.10              | Setup py_trees atau custom BT                   | ✅ DONE |
| 7.2.11              | Condition nodes (ball visible, near goal, etc.) | ✅ DONE |
| 7.2.12              | Action nodes (go to, kick, etc.)                | ✅ DONE |
| **Strategies**      |                                                 |         |
| 7.2.13              | Striker strategy                                | ✅ DONE |
| 7.2.14              | Goalkeeper strategy                             | ✅ DONE |
| 7.2.15              | Defender strategy                               | ⬜ TODO |
| 7.2.16              | Team coordination                               | ⬜ TODO |

#### 7.3 World Model

```python
# world_model/world_state.py
from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np

@dataclass
class RobotPose:
    x: float
    y: float
    theta: float
    confidence: float = 1.0

@dataclass
class BallState:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    is_visible: bool = True
    last_seen: float = 0.0

@dataclass
class WorldState:
    # Self state
    robot_pose: RobotPose

    # Ball
    ball: Optional[BallState] = None

    # Team
    teammates: List[RobotPose] = field(default_factory=list)

    # Opponents
    opponents: List[RobotPose] = field(default_factory=list)

    # Game state
    game_phase: str = "INITIAL"  # INITIAL, READY, SET, PLAYING, FINISHED
    own_score: int = 0
    opponent_score: int = 0

    # Field info
    own_goal_pos: tuple = (0, 0)
    opponent_goal_pos: tuple = (9, 0)

    def ball_distance(self) -> float:
        if self.ball is None:
            return float('inf')
        dx = self.ball.x - self.robot_pose.x
        dy = self.ball.y - self.robot_pose.y
        return np.sqrt(dx**2 + dy**2)

    def ball_angle(self) -> float:
        if self.ball is None:
            return 0.0
        dx = self.ball.x - self.robot_pose.x
        dy = self.ball.y - self.robot_pose.y
        return np.arctan2(dy, dx) - self.robot_pose.theta
```

#### 7.4 Behavior Tree Example

```python
# behavior_tree/striker.py
import py_trees

def create_striker_tree():
    """Create behavior tree for striker role"""

    root = py_trees.composites.Selector("Striker", memory=False)

    # Attack sequence
    attack = py_trees.composites.Sequence("Attack", memory=True)
    attack.add_children([
        IsBallVisible(),
        IsBallReachable(),
        py_trees.composites.Selector("GetBall", memory=False, children=[
            py_trees.composites.Sequence("HasBall", memory=True, children=[
                IsBallInDribbleRange(),
                AlignToGoal(),
                Kick()
            ]),
            py_trees.composites.Sequence("ChaseBall", memory=True, children=[
                ApproachBall(),
            ])
        ])
    ])

    # Search behavior
    search = py_trees.composites.Sequence("Search", memory=True)
    search.add_children([
        RotateToSearch(),
        MoveToSearchPosition()
    ])

    root.add_children([attack, search])

    return root
```

#### 7.5 Acceptance Criteria

- [ ] World model terupdate dengan latency < 50ms
- [ ] Behavior tree berjalan smooth tanpa stuttering
- [ ] Role assignment bekerja dengan benar
- [ ] Strategy dapat berganti berdasarkan game state
- [ ] Robot dapat bermain secara autonomous

---

## Integration Timeline

```
Week 1-2:   ╔══════════════════════════════════════╗
            ║  Phase 1: Foundation                  ║
            ║  • krsbi_msgs                         ║
            ║  • krsbi_description                  ║
            ║  • krsbi_interface                    ║
            ╚══════════════════════════════════════╝
                              │
Week 3-4:   ╔══════════════════════════════════════╗
            ║  Phase 2: Communication               ║
            ║  • krsbi_comm                         ║
            ║  • Arduino firmware integration       ║
            ╚══════════════════════════════════════╝
                              │
Week 5-7:   ╔══════════════════════════════════════╗
            ║  Phase 3: Perception                  ║
            ║  • krsbi_vision                       ║
            ║  • Camera calibration                 ║
            ║  • Detection tuning                   ║
            ╚══════════════════════════════════════╝
                              │
Week 8-10:  ╔══════════════════════════════════════╗
            ║  Phase 4: Control                     ║
            ║  • krsbi_control                      ║
            ║  • PID tuning                         ║
            ║  • Motion testing                     ║
            ╚══════════════════════════════════════╝
                              │
Week 11-13: ╔══════════════════════════════════════╗
            ║  Phase 5: Intelligence                ║
            ║  • krsbi_decision                     ║
            ║  • Strategy development               ║
            ║  • Behavior tuning                    ║
            ╚══════════════════════════════════════╝
                              │
Week 14-15: ╔══════════════════════════════════════╗
            ║  Phase 6: Integration                 ║
            ║  • Full system testing                ║
            ║  • Match simulation                   ║
            ║  • Bug fixing                         ║
            ╚══════════════════════════════════════╝
```

---

## Testing Strategy

### Unit Testing

Setiap package harus memiliki unit test di folder `test/`:

```bash
# Run tests untuk satu package
cd ~/ros2_ws
colcon test --packages-select krsbi_comm

# Run semua tests
colcon test
colcon test-result --all
```

### Integration Testing

1. **Hardware-in-the-Loop (HIL)**
   - Test komunikasi dengan Arduino
   - Test sensor readings
   - Test motor control

2. **Simulation Testing**
   - Setup Gazebo simulation
   - Test di simulated field

3. **Field Testing**
   - Test di lapangan sebenarnya
   - Match simulation

### Test Checklist

| Package           | Unit Test | Integration | Field Test |
| ----------------- | --------- | ----------- | ---------- |
| krsbi_msgs        | ⬜        | N/A         | N/A        |
| krsbi_description | ⬜        | ⬜          | N/A        |
| krsbi_interface   | ⬜        | ⬜          | N/A        |
| krsbi_comm        | ⬜        | ⬜          | ⬜         |
| krsbi_vision      | ⬜        | ⬜          | ⬜         |
| krsbi_control     | ⬜        | ⬜          | ⬜         |
| krsbi_decision    | ⬜        | ⬜          | ⬜         |

---

## Appendix

### A. Required ROS 2 Dependencies

```xml
<!-- Common dependencies -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>

<!-- Vision -->
<depend>cv_bridge</depend>
<depend>image_transport</depend>

<!-- Control -->
<depend>nav_msgs</depend>

<!-- Description -->
<depend>robot_state_publisher</depend>
<depend>joint_state_publisher</depend>
```

### B. Development Environment Setup

```bash
# Install ROS 2 Jazzy
# ...

# Create workspace
mkdir -p ~/krsbi_ws/src
cd ~/krsbi_ws/src

# Clone repository
git clone <repo_url>

# Install dependencies
cd ~/krsbi_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### C. Coding Standards

- Follow PEP 8 untuk Python
- Semua node harus memiliki:
  - Proper logging dengan `self.get_logger()`
  - Parameter declarations
  - Graceful shutdown handling
- Docstrings untuk semua functions dan classes
- Type hints untuk function signatures

---

**Last Updated:** 2026-02-06  
**Version:** 1.0.0  
**Maintainer:** KRSBI-B Team
