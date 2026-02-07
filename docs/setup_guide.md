# Setup Guide

This guide details the installation and configuration of the KRSBI-B ROS 2 Software Stack on Ubuntu 24.04 (ROS 2 Jazzy) or Compatible Linux.

## 📋 Prerequisites

### System Requirements

- **Operating System**: Ubuntu 24.04 LTS (Required for ROS 2 Jazzy).
- **Architecture**: x86_64 or ARM64 (Jetson Orin Nano / Raspberry Pi 5).
- **Memory**: Min 4GB RAM (8GB Recommended for Vision).
- **Storage**: 32GB SD Card / NVMe.

### Hardware Dependencies

- **Camera**: UVC Compatible Webcam or Intel RealSense via USB 3.0.
- **Microcontroller**: Arduino Mega 2560 or STM32F4 Discovery via USB Serial.
- **IMU**: MPU6050 via I2C to Microcontroller.
- **Motor Driver**: Cytron MD10C or L298N via PWM to Microcontroller.

## 📦 Installation Steps

### 1. Install ROS 2 Jazzy Jalisco

Follow official ROS 2 installation guide or use the convenient command below:

```bash
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

### 2. Install Python Dependencies

The system uses specific Python libraries for YOLO vision and Behavior Trees:

```bash
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install ultralytics opencv-python py_trees pyserial numpy
```

### 3. Create Workspace & Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-team/krsbi-ros2.git .
```

### 4. Install ROS Dependencies (rosdep)

Make sure all ROS packages relied upon are installed:

```bash
cd ~/ros2_ws
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Project

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## ⚙️ Configuration

### Serial Port Permissions (UDEV Rules)

To fix permission denied errors on Arduino (`/dev/ttyUSB0`):

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

_Ideally create a udev rule for persistent naming:_
`sudo nano /etc/udev/rules.d/99-arduino.rules`

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="arduino"
```

Then reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### Camera Configuration

Edit `krsbi_vision/config/camera_params.yaml` to match your camera device ID (`/dev/video0` or `/dev/video2`):

```yaml
camera:
  device_id: 0
  width: 640
  height: 480
```

### Network Configuration (Multi-Robot)

Set `ROS_DOMAIN_ID` unique to your team (0-100) in `.bashrc`:

```bash
export ROS_DOMAIN_ID=32
export ROS_LOCALHOST_ONLY=0
```

## ✅ Verification

Run the test suite to verify installation:

```bash
colcon test
```

If successful, you are ready to launch the robot!
Review `SOP_MATCH.md` for operational procedures.
