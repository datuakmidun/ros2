# GripperState.msg
# Status gripper robot (dikendalikan oleh driver BTS)

builtin_interfaces/Time stamp

# Position states
uint8 STATE_UNKNOWN = 0
uint8 STATE_OPEN = 1
uint8 STATE_CLOSED = 2
uint8 STATE_GRIPPING = 3           # Sedang mencengkram bola
uint8 STATE_RELEASING = 4          # Sedang melepas
uint8 STATE_MOVING = 5             # Sedang bergerak
uint8 state

# Position (jika ada feedback posisi)
float32 position                   # Posisi gripper (0.0 = fully closed, 1.0 = fully open)
float32 target_position            # Target posisi

# Motor info
int16 motor_pwm                    # PWM yang diberikan ke motor gripper (-255 to 255)
float32 motor_current              # Arus motor gripper (Ampere)

# Ball detection
bool ball_detected                 # Sensor mendeteksi bola di gripper
bool ball_secure                   # Bola tercengkram dengan aman

# Limit switches
bool limit_open                    # Limit switch posisi terbuka
bool limit_closed                  # Limit switch posisi tertutup

# Error
bool motor_stall                   # Motor gripper stall
bool error                         # Ada error pada gripper
string error_message
