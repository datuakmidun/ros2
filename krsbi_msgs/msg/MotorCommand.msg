# MotorCommand.msg
# Perintah untuk 3 motor PG45 (3-wheel omni configuration)
# Motor arrangement (top view):
#
#        FRONT (0°)
#           M1
#          /  \
#         /    \
#        M2----M3
#
# M1: Motor depan (front) - arah 0°
# M2: Motor kiri belakang (rear left) - arah 120°  
# M3: Motor kanan belakang (rear right) - arah 240°

builtin_interfaces/Time stamp

# Kecepatan motor dalam RPM (Rotations Per Minute)
# Nilai positif = arah forward sesuai orientasi motor
# Nilai negatif = arah backward
float64 motor1_rpm                 # Motor 1 (front)
float64 motor2_rpm                 # Motor 2 (rear left)
float64 motor3_rpm                 # Motor 3 (rear right)

# Alternatif: velocity command dalam m/s dan rad/s
# Akan dikonversi ke RPM oleh controller
float64 linear_x                   # Kecepatan linear x (m/s), positif = maju
float64 linear_y                   # Kecepatan linear y (m/s), positif = kiri
float64 angular_z                  # Kecepatan angular (rad/s), positif = CCW

# Mode command
uint8 MODE_VELOCITY = 0            # Gunakan linear_x, linear_y, angular_z
uint8 MODE_RPM = 1                 # Gunakan motor1_rpm, motor2_rpm, motor3_rpm
uint8 mode

# Safety
bool enable                        # Enable motor (false = brake)
bool emergency_stop                # Emergency stop (override semua)
