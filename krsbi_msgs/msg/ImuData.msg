# ImuData.msg
# Data dari sensor IMU (MPU6050/BNO055/etc)

builtin_interfaces/Time stamp

# Orientation dalam Euler angles (radian)
float64 roll                       # Roll angle (rad)
float64 pitch                      # Pitch angle (rad)
float64 yaw                        # Yaw angle (rad) - heading robot

# Orientation dalam quaternion (untuk transformasi yang lebih akurat)
geometry_msgs/Quaternion orientation

# Angular velocity (rad/s)
float64 angular_velocity_x         # Gyro X
float64 angular_velocity_y         # Gyro Y
float64 angular_velocity_z         # Gyro Z

# Linear acceleration (m/s^2)
float64 linear_acceleration_x      # Accel X
float64 linear_acceleration_y      # Accel Y
float64 linear_acceleration_z      # Accel Z

# Magnetic field (jika tersedia, dalam uT)
float64 magnetic_x
float64 magnetic_y
float64 magnetic_z
bool magnetometer_available

# Temperature sensor di IMU (Celsius)
float32 temperature

# Calibration status (khusus untuk BNO055)
uint8 calibration_system           # 0-3, 3 = fully calibrated
uint8 calibration_gyro
uint8 calibration_accel
uint8 calibration_mag
bool is_calibrated                 # True jika semua >= 1
