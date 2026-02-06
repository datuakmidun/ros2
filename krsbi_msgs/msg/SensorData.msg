# SensorData.msg
# Gabungan semua data sensor dari Arduino Mega

builtin_interfaces/Time stamp

# IMU Data
krsbi_msgs/ImuData imu

# Distance Sensors (Sharp GP)
krsbi_msgs/DistanceSensors distance_sensors

# Motor Feedback
krsbi_msgs/MotorFeedback motor_feedback

# Gripper State
krsbi_msgs/GripperState gripper

# Kicker State
krsbi_msgs/KickerState kicker

# Arduino Status
uint32 loop_time_us                # Waktu loop Arduino (microseconds)
bool watchdog_ok                   # Watchdog timer OK
uint8 error_count                  # Jumlah error komunikasi
