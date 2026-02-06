# RobotState.msg
# Status keseluruhan robot KRSBI-B

builtin_interfaces/Time stamp

# Pose robot di lapangan (field-centric coordinates)
geometry_msgs/Pose2D pose          # x, y (meter), theta (radian)

# Velocity saat ini
geometry_msgs/Twist velocity       # linear (m/s), angular (rad/s)

# Status sistem
bool is_ready                      # Robot siap bermain
bool is_localized                  # Lokalisasi valid
bool has_ball                      # Robot memiliki bola (di gripper)
bool is_moving                     # Robot sedang bergerak

# Battery status
float32 battery_voltage            # Tegangan battery (Volt)
float32 battery_percentage         # Persentase battery (0-100)
bool battery_low                   # Warning battery rendah

# Kicker status
bool kicker_ready                  # Kicker siap (capacitor charged)
float32 kicker_charge_level        # Level charge capacitor (0-100%)

# Gripper status
bool gripper_open                  # Status gripper terbuka
bool gripper_has_ball              # Bola terdeteksi di gripper

# Hardware status
bool motors_ok                     # Semua motor berfungsi
bool sensors_ok                    # Semua sensor berfungsi
bool communication_ok              # Komunikasi dengan Arduino OK

# Current behavior
string current_behavior            # Nama behavior yang sedang dijalankan
string current_role                # Role saat ini (striker/goalkeeper/defender)

# Error codes
uint8 NO_ERROR = 0
uint8 ERROR_MOTOR = 1
uint8 ERROR_SENSOR = 2
uint8 ERROR_COMMUNICATION = 3
uint8 ERROR_BATTERY = 4
uint8 ERROR_KICKER = 5
uint8 error_code                   # Kode error jika ada
string error_message               # Pesan error detail
