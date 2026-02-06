# krsbi_interface

High-level interface, configuration, and utilities for KRSBI-B Soccer Robot.

## 📋 Overview

This package provides:

- **Configuration files** for all robot subsystems
- **Constants and utilities** for common operations
- **Launch files** for different operating modes
- **System monitoring** for diagnostics

## 📁 Package Structure

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
│   ├── robot_bringup.launch.py    # Main robot startup
│   ├── match.launch.py            # Competition mode
│   └── simulation.launch.py       # Simulation/testing mode
├── package.xml
├── setup.py
├── README.md
└── CHANGELOG.md
```

## 🚀 Usage

### Launch Robot

```bash
# Build
cd ~/krsbi_ws
colcon build --packages-select krsbi_interface
source install/setup.bash

# Basic bringup
ros2 launch krsbi_interface robot_bringup.launch.py

# With arguments
ros2 launch krsbi_interface robot_bringup.launch.py \
    robot_id:=1 \
    team_color:=blue \
    role:=striker

# Match mode
ros2 launch krsbi_interface match.launch.py robot_id:=1 team_color:=blue

# Simulation mode
ros2 launch krsbi_interface simulation.launch.py gui:=true
```

### Access Parameters

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare and get parameters
        self.declare_parameter('robot.robot_id', 1)
        robot_id = self.get_parameter('robot.robot_id').value
```

### Use Constants

```python
from krsbi_interface import (
    GameState, RobotRole, TeamColor,
    FieldDimensions, RobotLimits,
    Topics, Services, Actions
)

# Check game state
if game_state == GameState.PLAYING:
    # Game is active
    pass

# Get field dimensions
half_length = FieldDimensions.HALF_LENGTH  # 4.5 meters

# Check robot limits
if velocity > RobotLimits.MAX_LINEAR_VELOCITY:
    velocity = RobotLimits.MAX_LINEAR_VELOCITY

# Use standard topic names
ball_topic = Topics.BALL_POSITION  # "/krsbi/vision/ball"
```

### Use Utilities

```python
from krsbi_interface import (
    normalize_angle, angle_difference,
    distance_2d, point_in_field,
    clamp, map_value
)

# Normalize angle to [-π, π]
angle = normalize_angle(4.0)  # Returns ~-2.28

# Calculate distance
dist = distance_2d(0, 0, 3, 4)  # Returns 5.0

# Clamp value
speed = clamp(2.0, 0.0, 1.5)  # Returns 1.5

# Check if point is in field
if point_in_field(x, y):
    # Point is inside field boundaries
    pass
```

## ⚙️ Configuration Files

### robot_params.yaml

```yaml
robot:
  robot_id: 1
  team_color: "blue"
  role: "striker"

hardware:
  serial:
    port: "/dev/ttyUSB0"
    baudrate: 115200

limits:
  max_linear_velocity: 1.5 # m/s
  max_angular_velocity: 3.14 # rad/s
```

### vision_params.yaml

```yaml
cameras:
  omni:
    device_id: 0
    resolution: [640, 480]
    fps: 30

ball_detection:
  color:
    hue_low: 5
    hue_high: 25
    saturation_low: 100
```

### control_params.yaml

```yaml
motion:
  max_linear_accel: 2.0 # m/s²
  deadband:
    linear: 0.03 # m/s

kinematics:
  wheel_radius: 0.05 # meters
  robot_radius: 0.17 # meters
```

### game_params.yaml

```yaml
field:
  length: 9.0 # meters
  width: 6.0 # meters
  goal_width: 2.6 # meters

timing:
  half_duration: 600 # seconds
```

## 📊 System Monitor

The system monitor publishes diagnostics to `/diagnostics`:

```bash
# View diagnostics
ros2 topic echo /diagnostics

# View simple status
ros2 topic echo /krsbi/system_status
```

Monitored components:

- CPU usage and temperature
- Memory usage
- Battery status
- Communication health
- Sensor status

## 🔧 Constants Reference

### Game States

| State    | Value | Description        |
| -------- | ----- | ------------------ |
| INITIAL  | 0     | Before game starts |
| READY    | 1     | Move to positions  |
| SET      | 2     | Wait for kickoff   |
| PLAYING  | 3     | Game active        |
| FINISHED | 4     | Game ended         |

### Robot Roles

| Role       | Value |
| ---------- | ----- |
| GOALKEEPER | 0     |
| STRIKER    | 1     |
| DEFENDER   | 2     |
| SUPPORT    | 3     |

### Topic Names

| Topic         | Name                   |
| ------------- | ---------------------- |
| Sensor Data   | `/krsbi/sensor_data`   |
| Motor Command | `/krsbi/motor_command` |
| Ball Position | `/krsbi/vision/ball`   |
| Robot State   | `/krsbi/robot_state`   |
| Game State    | `/krsbi/game_state`    |

## 📝 Dependencies

- `rclpy` - ROS 2 Python library
- `std_msgs` - Standard messages
- `diagnostic_msgs` - Diagnostics
- `krsbi_msgs` - Custom messages
- `krsbi_description` - Robot model
- `psutil` - System monitoring (Python)

## 📄 License

MIT License - See LICENSE file for details.

## 👥 Authors

- KRSBI-B Team
