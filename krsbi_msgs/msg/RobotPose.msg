# RobotPose.msg
# Pose robot dalam koordinat lapangan (field frame)
# Origin: Tengah lapangan
# X: Positif ke arah gawang lawan
# Y: Positif ke kiri (dari sudut pandang menghadap gawang lawan)

builtin_interfaces/Time stamp

# Robot identifier
uint8 robot_id                     # ID robot (1-4)
uint8 team                         # 0 = Blue, 1 = Yellow

# Pose
float64 x                          # Posisi x (meter)
float64 y                          # Posisi y (meter)
float64 theta                      # Heading (radian), 0 = menghadap gawang lawan

# Velocity
float64 vx                         # Kecepatan linear x (m/s)
float64 vy                         # Kecepatan linear y (m/s)
float64 vtheta                     # Kecepatan angular (rad/s)

# Uncertainty (covariance)
float64[9] covariance              # 3x3 covariance matrix [xx, xy, xt, yx, yy, yt, tx, ty, tt]

# Confidence
float64 confidence                 # 0.0 - 1.0

# Status
bool is_valid                      # Pose valid
bool is_fallen                     # Robot jatuh
bool is_penalized                  # Robot dalam penalti

# Source
uint8 SOURCE_SELF = 0              # Pose sendiri (dari lokalisasi)
uint8 SOURCE_TEAMMATE = 1          # Dikirim oleh teman
uint8 SOURCE_VISION = 2            # Terlihat oleh robot lain
uint8 source
