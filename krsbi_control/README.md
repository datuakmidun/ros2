# krsbi_control

Motion control, kinematics, and path planning package for KRSBI-B Soccer Robot.

## 📋 Overview

This package handles all robot movement logic:

- **Motion Control**: Velocity ramping, limits, and PID for motors.
- **Kinematics**: 3-wheel Omni-directional kinematics (Forward/Inverse).
- **Localization**: Odometry calculation using wheel encoders and IMU fusion.
- **Path Planning**: Local path planning with obstacle avoidance.
- **Behaviors**: High-level skills (Follow Ball, Go To Position).

## 📁 Package Structure

```
krsbi_control/
├── krsbi_control/
│   ├── __init__.py
│   ├── motion_controller.py     # Handling cmd_vel, ramping
│   ├── omni_kinematics.py       # 3-wheel omni math
│   ├── pid_controller.py        # Discrete PID implementation
│   ├── localization.py          # Odometry & IMU fusion
│   ├── path_planner.py          # Go-to-goal & obstacle avoidance
│   ├── trajectory_tracker.py    # Waypoint following (WIP)
│   ├── behavior_node.py         # Behavior state machine
│   └── behaviors/               # Skill implementations
├── config/
│   └── control_config.yaml      # PID, limits, kinematics params
├── launch/
│   └── control_bringup.launch.py
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Usage

### Launch Control System

```bash
# Build
colcon build --packages-select krsbi_control
source install/setup.bash

# Launch full stack
ros2 launch krsbi_control control_bringup.launch.py

# With IMU disabled
ros2 launch krsbi_control control_bringup.launch.py use_imu:=false
```

### Run Individual Nodes

```bash
# Motion Controller
ros2 run krsbi_control motion_controller

# Localization (Odometry)
ros2 run krsbi_control localization

# Path Planner
ros2 run krsbi_control path_planner
```

## 🕹️ Control Interfaces

### Motion Command

Send velocity commands to `/cmd_vel` (geometry_msgs/Twist).
The `motion_controller` will ramp and limit these before sending to hardware.

### Navigation Goal

Send goal pose to `/move_base_simple/goal` (geometry_msgs/PoseStamped).
The `path_planner` will drive the robot to this point, avoiding obstacles.

### Behaviors

Publish behavior name to `/krsbi/behavior/command` (std_msgs/String):

- `FOLLOW_BALL`: Chase the detected ball.
- `DRIBBLE`: Dribble ball (WIP).
- `IDLE`: Stop and wait.

## ⚙️ Configuration (control_config.yaml)

### Limits

```yaml
motion:
  ramping:
    max_linear_accel: 2.0 # m/s²
    max_angular_accel: 6.28 # rad/s²
```

### PID gains

```yaml
pid:
  kp: 1.2
  ki: 0.5
  kd: 0.05
```

### Kinematics

```yaml
kinematics:
  wheel_radius: 0.05
  robot_radius: 0.17
  wheel_angles: [0, 120, 240]
```

## 📐 Coordinate Frames

- `odom`: World-fixed frame (starts at 0,0).
- `base_footprint`: Projection of robot center on ground.
- `base_link`: Robot center.

TF Tree: `odom -> base_footprint -> base_link`

## 📦 Dependencies

- `krsbi_msgs`
- `krsbi_interface`
- `tf2_ros`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs` (IMU)
