# Changelog

All notable changes to the `krsbi_interface` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-06

### Added

#### Configuration Files

- **robot_params.yaml** - Robot identity, hardware config, motion limits, PID parameters, battery settings
- **vision_params.yaml** - Camera configuration, ball/robot/goal detection, image processing settings
- **control_params.yaml** - Motion control, kinematics, path planning, obstacle avoidance, behaviors
- **game_params.yaml** - Field dimensions, game timing, roles, zones, GameController settings

#### Python Modules

- **constants.py** - Game states, robot roles, team colors, penalties, field dimensions, robot limits, topic/service/action names
- **utils.py** - Utility functions for angles, geometry, conversions, team logic
- **system_monitor.py** - Node for monitoring CPU, memory, battery, communication, and sensors
- **param_server.py** - Node for loading and providing centralized parameter access

#### Launch Files

- **robot_bringup.launch.py** - Main robot startup with param_server and system_monitor
- **match.launch.py** - Competition mode launch file
- **simulation.launch.py** - Simulation/testing mode launch file

### Feature Highlights

- Comprehensive configuration for all robot subsystems
- Constants matching krsbi_msgs definitions
- Utility functions for common robot operations
- System health monitoring with diagnostics
- Launch files for different operating modes

### Enumerations

- `GameState` - INITIAL, READY, SET, PLAYING, FINISHED
- `SetPlay` - NORMAL, KICKOFF, FREEKICK, PENALTY, CORNER, etc.
- `RobotRole` - GOALKEEPER, STRIKER, DEFENDER, SUPPORT
- `TeamColor` - BLUE, YELLOW
- `MotorID` - FRONT, REAR_LEFT, REAR_RIGHT
- `CameraSource` - OMNI, FRONT, FUSED
- `KickerState` - IDLE, CHARGING, READY, KICKING, COOLDOWN, ERROR
- `GripperAction` - OPEN, CLOSE, GRAB, RELEASE, STOP, SET_POSITION
- `PowerLevel` - LOW, MEDIUM, HIGH, MAX

### Data Classes

- `FieldDimensions` - KRSBI-B field specifications
- `RobotLimits` - Physical and operational constraints
- `WheelConfig` - 3-wheel omni configuration
- `BallSpec` - Ball specifications
- `TimingConfig` - Game timing
- `SerialConfig` - Serial communication defaults
- `GameControllerConfig` - Network settings

---

## [Unreleased]

### Planned

- GameController client node
- Team communication node
- Parameter validation
- Dynamic reconfigure support

---

## Version History

| Version | Date       | Description                                                         |
| ------- | ---------- | ------------------------------------------------------------------- |
| 1.0.0   | 2026-02-06 | Initial release - Configuration, constants, utilities, launch files |
