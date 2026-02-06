# krsbi_msgs

Custom ROS 2 messages, services, dan actions untuk robot KRSBI-B Soccer.

## Deskripsi

Package ini berisi definisi interface untuk komunikasi antar-node pada sistem robot sepak bola KRSBI-B. Semua message disesuaikan dengan hardware robot:

- **2 Kamera**: Omni-directional fisheye 360° + Logitech webcam
- **3 Motor**: PG45 dengan encoder (3-wheel omni configuration)
- **Sensor**: Sharp GP distance sensors, IMU
- **Aktuator**: Gripper dengan driver BTS, Kicker dengan 2 relay

## Struktur

```
krsbi_msgs/
├── msg/
│   ├── BallPosition.msg      # Posisi bola terdeteksi
│   ├── RobotState.msg        # Status keseluruhan robot
│   ├── MotorCommand.msg      # Perintah ke motor
│   ├── MotorFeedback.msg     # Feedback dari encoder
│   ├── SensorData.msg        # Gabungan semua sensor
│   ├── ImuData.msg           # Data IMU
│   ├── DistanceSensors.msg   # Data sensor jarak Sharp GP
│   ├── GripperState.msg      # Status gripper
│   ├── KickerState.msg       # Status kicker/penendang
│   ├── GameState.msg         # Status permainan
│   ├── DetectedObject.msg    # Objek terdeteksi (bola/robot/gawang)
│   ├── DetectedObjects.msg   # Kumpulan objek terdeteksi
│   ├── FieldLines.msg        # Garis lapangan terdeteksi
│   ├── RobotPose.msg         # Pose robot di lapangan
│   └── TeamRobots.msg        # Info semua robot dalam tim
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

## Build

```bash
# Di workspace root
cd ~/krsbi_ws

# Build package
colcon build --packages-select krsbi_msgs

# Source
source install/setup.bash
```

## Penggunaan

### Python

```python
from krsbi_msgs.msg import BallPosition, MotorCommand, RobotState
from krsbi_msgs.srv import SetMotorSpeed, Kick
from krsbi_msgs.action import MoveTo, GrabBall

# Contoh publish ball position
ball_msg = BallPosition()
ball_msg.x = 1.5
ball_msg.y = 0.3
ball_msg.distance = 1.53
ball_msg.angle = 0.197
ball_msg.confidence = 0.95
ball_msg.is_visible = True
ball_msg.camera_source = BallPosition.CAMERA_OMNI
```

### C++

```cpp
#include "krsbi_msgs/msg/ball_position.hpp"
#include "krsbi_msgs/msg/motor_command.hpp"
#include "krsbi_msgs/srv/kick.hpp"
#include "krsbi_msgs/action/move_to.hpp"

// Contoh
auto ball_msg = krsbi_msgs::msg::BallPosition();
ball_msg.x = 1.5;
ball_msg.y = 0.3;
ball_msg.is_visible = true;
```

## Message Details

### BallPosition

Posisi bola relatif terhadap robot. Mendukung koordinat kartesian dan polar. Menyertakan informasi sumber kamera (OMNI/FRONT/FUSED).

### MotorCommand

Perintah untuk 3 motor PG45. Mendukung mode velocity (m/s, rad/s) dan mode RPM langsung.

```
Motor arrangement (top view):
       FRONT (0°)
          M1
         /  \
        /    \
       M2----M3
```

### KickerState

Status sistem penendang dengan capacitor dan 2 relay:

- Relay 1: Charging (mengisi capacitor)
- Relay 2: Discharge (melepas ke solenoid)

**Safety**: Kedua relay TIDAK BOLEH aktif bersamaan!

### GameState

Status permainan sesuai protokol GameController KRSBI-B / RoboCup.

## Dependencies

- `std_msgs`
- `geometry_msgs`
- `builtin_interfaces`

## License

MIT License

## Maintainer

- datuakmidun (datuakmidun@gmail.com)
