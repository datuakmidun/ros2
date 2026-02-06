# DetectedObject.msg
# Objek yang terdeteksi oleh sistem vision
# Digunakan untuk bola, robot, gawang, dll

builtin_interfaces/Time stamp

# Object type
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_BALL = 1
uint8 TYPE_ROBOT_TEAMMATE = 2
uint8 TYPE_ROBOT_OPPONENT = 3
uint8 TYPE_ROBOT_UNKNOWN = 4
uint8 TYPE_GOALPOST_OWN = 5
uint8 TYPE_GOALPOST_OPP = 6
uint8 TYPE_GOALPOST_UNKNOWN = 7
uint8 TYPE_FIELD_LINE = 8
uint8 TYPE_FIELD_CORNER = 9
uint8 TYPE_CENTER_CIRCLE = 10
uint8 TYPE_PENALTY_MARK = 11
uint8 object_type

# Position dalam robot frame (meter)
float64 x                          # Jarak depan (positif = depan)
float64 y                          # Jarak samping (positif = kiri)
float64 z                          # Tinggi (untuk objek 3D)

# Position dalam polar coordinates
float64 distance                   # Jarak langsung (meter)
float64 angle                      # Sudut dari depan robot (radian)

# Bounding box dalam image (pixel)
int32 bbox_x                       # Top-left x
int32 bbox_y                       # Top-left y
int32 bbox_width                   # Width
int32 bbox_height                  # Height

# Size estimation
float64 estimated_size             # Perkiraan ukuran objek (meter)

# Detection confidence
float64 confidence                 # 0.0 - 1.0

# Tracking ID (untuk tracking antar frame)
int32 tracking_id                  # -1 jika tidak di-track

# Camera source
uint8 CAMERA_OMNI = 0
uint8 CAMERA_FRONT = 1
uint8 camera_source

# Additional info untuk robot
int32 robot_number                 # Nomor robot (jika diketahui)
uint8 team_color                   # 0 = Blue, 1 = Yellow, 255 = Unknown
