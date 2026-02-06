# MotorFeedback.msg
# Feedback dari 3 motor PG45 dengan encoder

builtin_interfaces/Time stamp

# Encoder counts (raw value dari encoder)
int32 encoder1_ticks               # Motor 1 encoder ticks
int32 encoder2_ticks               # Motor 2 encoder ticks
int32 encoder3_ticks               # Motor 3 encoder ticks

# RPM aktual (dihitung dari encoder)
float64 motor1_rpm                 # Motor 1 actual RPM
float64 motor2_rpm                 # Motor 2 actual RPM
float64 motor3_rpm                 # Motor 3 actual RPM

# PWM yang diberikan ke motor (untuk debugging)
int16 motor1_pwm                   # Motor 1 PWM (-255 to 255)
int16 motor2_pwm                   # Motor 2 PWM (-255 to 255)
int16 motor3_pwm                   # Motor 3 PWM (-255 to 255)

# Current/Arus motor (jika ada sensor arus)
float32 motor1_current             # Motor 1 current (Ampere)
float32 motor2_current             # Motor 2 current (Ampere)
float32 motor3_current             # Motor 3 current (Ampere)

# Status motor
bool motor1_enabled
bool motor2_enabled
bool motor3_enabled

# Error flags
bool motor1_stall                  # Motor 1 stall detected
bool motor2_stall                  # Motor 2 stall detected
bool motor3_stall                  # Motor 3 stall detected
