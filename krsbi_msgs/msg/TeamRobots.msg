# TeamRobots.msg
# Informasi semua robot dalam tim (untuk koordinasi)

builtin_interfaces/Time stamp

# Team info
uint8 team_color                   # 0 = Blue, 1 = Yellow
uint8 robot_count                  # Jumlah robot aktif

# All robot poses (max 4 robot untuk KRSBI-B)
krsbi_msgs/RobotPose[4] robots

# Robot roles
uint8 ROLE_UNDEFINED = 0
uint8 ROLE_STRIKER = 1
uint8 ROLE_GOALKEEPER = 2
uint8 ROLE_DEFENDER = 3
uint8 ROLE_SUPPORTER = 4
uint8[4] roles                     # Role masing-masing robot

# Robot status
bool[4] active                     # Robot dalam permainan
bool[4] has_ball                   # Robot memiliki bola
bool[4] sees_ball                  # Robot melihat bola

# Ball info from team perspective
bool team_sees_ball                # Setidaknya satu robot melihat bola
krsbi_msgs/BallPosition team_ball  # Posisi bola dari perspektif tim (fused)

# Striker assignment
uint8 current_striker_id           # ID robot yang menjadi striker utama
float64[4] ball_distances          # Jarak masing-masing robot ke bola
