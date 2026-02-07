# Changelog

All notable changes to the `krsbi_control` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-07

### Added

#### Motion Control

- **motion_controller.py**: Implements velocity ramping and safety limits.
- **pid_controller.py**: Discrete PID controller with integral windup protection.
- **omni_kinematics.py**: 3-wheel Omni-directional kinematics (Forward/Inverse).

#### Localization

- **localization.py**: Odometry calculation using wheel encoders and IMU fusion.
- Supports 3-wheel omni odometry with configurable noise.
- Publishes `/odom` and broadcasts TF `odom -> base_footprint`.

#### Path Planning

- **path_planner.py**: Simple point-to-point planner with local obstacle avoidance.
- Subscribes to `/move_base_simple/goal` (Compatible with RViz).
- Uses simple potential field or rule-based avoidance.

#### Behaviors

- **behavior_node.py**: Manages robot behaviors (FollowBall, GoTo).
- Implements basic state machine for robot actions.
- `trajectory_tracker.py` (Placeholder): Pure pursuit for future use.

#### Configuration

- **control_config.yaml**: Centralized configuration for PID, limits, kinematics.
- **control_bringup.launch.py**: Launch file for full control stack.

### Notes

- Requires `krsbi_msgs` for full functionality (IMU, distance sensors).
- Coordinate system follows standard ROS conventions (X forward, Y left).
