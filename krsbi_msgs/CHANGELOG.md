# Changelog

All notable changes to the `krsbi_msgs` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-06

### Added

#### Messages (15 files)

- **BallPosition.msg** - Ball position detection with Cartesian/polar coordinates, velocity estimation, and dual camera support (OMNI/FRONT/FUSED)
- **RobotState.msg** - Comprehensive robot state including pose, velocity, battery, kicker, gripper status, and error codes
- **MotorCommand.msg** - Motor command for 3-wheel omni robot with velocity (m/s) and RPM modes
- **MotorFeedback.msg** - Encoder feedback with ticks, actual RPM, PWM values, and stall detection
- **SensorData.msg** - Composite message aggregating all sensor data from Arduino
- **ImuData.msg** - IMU data with Euler angles, quaternion, angular velocity, acceleration, and calibration status
- **DistanceSensors.msg** - Sharp GP infrared distance sensor readings with obstacle detection flags
- **GripperState.msg** - Gripper status with position, ball detection, limit switches, and motor info
- **KickerState.msg** - Kicker system status with capacitor voltage, relay states, power levels, and safety checks
- **GameState.msg** - Game state following RoboCup/KRSBI-B GameController protocol (phases, set plays, penalties)
- **DetectedObject.msg** - Generic detected object (ball, robot, goalpost, field features) with position and tracking
- **DetectedObjects.msg** - Collection of detected objects with quick access counts
- **FieldLines.msg** - Detected field lines for localization (center circle, corners, intersections)
- **RobotPose.msg** - Robot pose in field coordinates with velocity and uncertainty covariance
- **TeamRobots.msg** - Team coordination info with roles, ball possession, and striker assignment

#### Services (7 files)

- **SetMotorSpeed.srv** - Set motor speed with velocity or RPM mode
- **SetGripper.srv** - Control gripper (open, close, grab, release, set position)
- **ChargeKicker.srv** - Charge kicker capacitor to target voltage/percentage
- **Kick.srv** - Execute kick with power level or discharge duration
- **GetRobotState.srv** - Get comprehensive robot state with optional sensor data
- **Calibrate.srv** - Calibrate sensors (IMU, motors, distance, kicker, gripper)
- **SetGameState.srv** - Set game state manually or from GameController

#### Actions (4 files)

- **MoveTo.action** - Move robot to target position with obstacle avoidance
- **RotateTo.action** - Rotate robot to target heading with direction control
- **KickBall.action** - Complete kick sequence (approach, align, charge, kick)
- **GrabBall.action** - Grab ball with gripper (search, approach, grab, secure)

#### Documentation

- **README.md** - Package documentation with structure, usage examples, and message details
- **CHANGELOG.md** - This changelog file

#### Build System

- **CMakeLists.txt** - CMake configuration for message generation with rosidl
- **package.xml** - Package manifest with ament_cmake build type and dependencies

### Hardware Support

- 3-wheel omni-directional robot (PG45 motors with encoders)
- Dual camera system (Omni fisheye 360° + Logitech webcam)
- Sharp GP infrared distance sensors
- IMU sensor (MPU6050/BNO055 compatible)
- Gripper with BTS7960 driver
- Kicker with capacitor charging system (2 relays)

### Dependencies

- `std_msgs`
- `geometry_msgs`
- `builtin_interfaces`
- `rosidl_default_generators`

---

## [Unreleased]

### Planned

- Unit tests for message serialization
- Integration tests with other packages
- Python helper utilities for message creation

---

## Version History

| Version | Date       | Description                                                         |
| ------- | ---------- | ------------------------------------------------------------------- |
| 1.0.0   | 2026-02-06 | Initial release - Complete message, service, and action definitions |
