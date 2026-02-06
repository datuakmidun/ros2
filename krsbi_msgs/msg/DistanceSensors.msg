# DistanceSensors.msg
# Data dari sensor jarak Sharp GP
# Posisi sensor disesuaikan dengan konfigurasi robot

builtin_interfaces/Time stamp

# Jarak dalam meter
# Sharp GP2Y0A21 range: 10-80cm
# Sharp GP2Y0A02 range: 20-150cm

# Front sensors
float32 front_left                 # Sensor depan kiri (meter)
float32 front_center               # Sensor depan tengah (meter)
float32 front_right                # Sensor depan kanan (meter)

# Side sensors
float32 left                       # Sensor samping kiri (meter)
float32 right                      # Sensor samping kanan (meter)

# Rear sensors
float32 rear_left                  # Sensor belakang kiri (meter)
float32 rear_center                # Sensor belakang tengah (meter)
float32 rear_right                 # Sensor belakang kanan (meter)

# Array format untuk kemudahan processing
# Urutan: FL, FC, FR, L, R, RL, RC, RR
float32[8] distances

# Valid flags (sensor dalam range valid)
bool[8] valid

# Obstacle detection (jarak < threshold)
bool obstacle_front
bool obstacle_left
bool obstacle_right
bool obstacle_rear

# Closest obstacle
float32 closest_distance           # Jarak obstacle terdekat (meter)
float32 closest_angle              # Sudut obstacle terdekat (radian)
