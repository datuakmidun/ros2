# krsbi_description

URDF robot description and visualization package for KRSBI-B Soccer Robot.

## 📋 Overview

This package provides the robot model (URDF/Xacro), visualization tools, and configuration files for the KRSBI-B Soccer Robot. It includes:

- **URDF/Xacro files** defining the robot structure
- **Launch files** for RViz visualization
- **Config files** for robot parameters
- **RViz configurations** for display

## 🤖 Robot Model

### Physical Specifications

| Component        | Specification                                   |
| ---------------- | ----------------------------------------------- |
| **Base**         | Cylindrical, 40cm diameter, 30cm height         |
| **Wheels**       | 3x Omni wheels (10cm diameter) with PG45 motors |
| **Drive**        | 3-wheel omni-directional                        |
| **Total Height** | ~40cm (including camera mount)                  |
| **Mass**         | ~10kg (estimated)                               |

### Wheel Configuration (Top View)

```
        FRONT (0°)
           M1
          /  \
         /    \
        M2----M3
    (120°)   (240°)
```

### Sensors

| Sensor            | Location             | Description                  |
| ----------------- | -------------------- | ---------------------------- |
| **Omni Camera**   | Top center (35cm)    | Fisheye 360° omnidirectional |
| **Front Camera**  | Front (18cm, 25cm H) | Logitech webcam, ~12° pitch  |
| **IMU**           | Center (15cm H)      | MPU6050/BNO055               |
| **Distance (6x)** | Around perimeter     | Sharp GP IR sensors          |

### Actuators

| Actuator    | Description                        |
| ----------- | ---------------------------------- |
| **Gripper** | Prismatic arms with ball detection |
| **Kicker**  | Solenoid with capacitor charging   |

## 📁 Package Structure

```
krsbi_description/
├── krsbi_description/
│   ├── __init__.py
│   └── state_publisher.py      # Custom joint state publisher
├── urdf/
│   ├── robot.urdf.xacro        # Main robot description
│   ├── materials.xacro         # Color/material definitions
│   ├── properties.xacro        # Physical properties/dimensions
│   ├── wheel.xacro             # Omni wheel macro
│   ├── sensors.xacro           # Camera, IMU, distance sensors
│   └── actuators.xacro         # Gripper and kicker
├── config/
│   ├── robot_params.yaml       # Robot parameters
│   └── joint_limits.yaml       # Joint limit configuration
├── launch/
│   ├── display.launch.py       # RViz visualization
│   ├── robot_state_publisher.launch.py
│   └── view_frames.launch.py   # TF debugging
├── rviz/
│   └── display.rviz            # RViz configuration
├── meshes/                     # STL/DAE mesh files (if available)
├── package.xml
└── setup.py
```

## 🚀 Usage

### Display Robot in RViz

```bash
# Build the package
cd ~/krsbi_ws
colcon build --packages-select krsbi_description
source install/setup.bash

# Launch visualization
ros2 launch krsbi_description display.launch.py

# With joint state publisher GUI
ros2 launch krsbi_description display.launch.py use_gui:=true
```

### Robot State Publisher Only

```bash
# Include in other launch files or run standalone
ros2 launch krsbi_description robot_state_publisher.launch.py
```

### View TF Frames

```bash
# Launch robot and view frames
ros2 launch krsbi_description view_frames.launch.py

# Generate TF tree PDF
ros2 run tf2_tools view_frames
```

### Check URDF Validity

```bash
# Check for URDF errors
ros2 run xacro xacro urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

## 🔗 TF Frames

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── front_wheel_link
            ├── rear_left_wheel_link
            ├── rear_right_wheel_link
            ├── omni_camera_link
            │   └── omni_camera_optical_frame
            ├── front_camera_link
            │   └── front_camera_optical_frame
            ├── imu_link
            ├── distance_front_link
            ├── distance_front_left_link
            ├── distance_front_right_link
            ├── distance_left_link
            ├── distance_right_link
            ├── distance_rear_link
            ├── gripper_base_link
            │   ├── gripper_left_link
            │   ├── gripper_right_link
            │   └── gripper_sensor_link
            └── kicker_housing_link
                └── kicker_link
```

## 🔧 Configuration

### Robot Parameters (`config/robot_params.yaml`)

```yaml
robot:
  name: "krsbi_robot"
  namespace: "krsbi"

wheels:
  type: "omni_3"
  count: 3
  max_rpm: 200

cameras:
  omni:
    type: "fisheye_360"
    fov: 360
  front:
    type: "logitech_webcam"
    fov: 78
```

### Customizing Dimensions

Edit `urdf/properties.xacro` to match your robot's actual dimensions:

```xml
<xacro:property name="base_radius" value="0.20"/>
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_base_distance" value="0.17"/>
```

## 🔗 Integration with Other Packages

### With krsbi_msgs

The state_publisher node subscribes to:

- `/krsbi/motor_feedback` → Updates wheel joint positions
- `/krsbi/gripper_state` → Updates gripper joint positions
- `/krsbi/kicker_state` → Updates kicker joint position

### With krsbi_comm

The robot state publisher receives actual sensor data from Arduino via krsbi_comm.

## 📝 Dependencies

- `rclpy` - ROS 2 Python library
- `robot_state_publisher` - TF broadcaster
- `joint_state_publisher` - Joint state simulation
- `joint_state_publisher_gui` - GUI for manual control
- `xacro` - XML macro for URDF
- `rviz2` - Visualization
- `tf2_ros` - Transform library

## 📄 License

MIT License - See LICENSE file for details.

## 👥 Authors

- KRSBI-B Team
