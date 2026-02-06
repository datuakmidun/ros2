# Changelog

All notable changes to the `krsbi_description` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-06

### Added

#### URDF/Xacro Files

- **robot.urdf.xacro** - Main robot description with 3-wheel omni configuration
- **materials.xacro** - Material and color definitions for visualization
- **properties.xacro** - Physical properties, dimensions, and inertia macros
- **wheel.xacro** - Omni wheel macro with continuous joints
- **sensors.xacro** - Sensor definitions:
  - Omni camera (fisheye 360°) with Gazebo plugin
  - Front camera (Logitech webcam) with Gazebo plugin
  - IMU sensor with Gazebo plugin
  - Sharp GP distance sensors (6x) with ray sensor plugin
- **actuators.xacro** - Actuator definitions:
  - Gripper with prismatic joints and mimic
  - Kicker with solenoid housing and prismatic joint

#### Launch Files

- **display.launch.py** - RViz visualization with joint state publisher GUI
- **robot_state_publisher.launch.py** - Standalone robot state publisher
- **view_frames.launch.py** - TF frame debugging

#### Configuration

- **robot_params.yaml** - Robot physical and operational parameters
- **joint_limits.yaml** - Joint limits for wheels, gripper, and kicker

#### Visualization

- **display.rviz** - RViz configuration for robot model display

#### Python Nodes

- **state_publisher.py** - Custom joint state publisher that converts sensor feedback to joint states

### Robot Specifications

- 3-wheel omni-directional drive (PG45 motors)
- Cylindrical base (40cm diameter, 30cm height)
- Dual camera system (omni fisheye + Logitech webcam)
- 6x Sharp GP distance sensors
- IMU sensor (MPU6050/BNO055 compatible)
- Gripper with ball detection
- Kicker with capacitor charging system

### Coordinate System

- X: Forward (positive = front)
- Y: Left (positive = left side)
- Z: Up (positive = upward)

---

## [Unreleased]

### Planned

- STL mesh files from CAD model
- Gazebo simulation launch files
- Collision geometry optimization
- Sensor noise models for simulation

---

## Version History

| Version | Date       | Description                           |
| ------- | ---------- | ------------------------------------- |
| 1.0.0   | 2026-02-06 | Initial release - Complete URDF model |
