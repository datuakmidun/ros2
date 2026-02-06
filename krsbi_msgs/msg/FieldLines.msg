# FieldLines.msg
# Garis lapangan yang terdeteksi untuk lokalisasi

builtin_interfaces/Time stamp

# Detected line segments dalam robot frame
# Format: [x1, y1, x2, y2] untuk setiap segment
float64[] line_segments            # Array of line segments (x1,y1,x2,y2,x1,y1,...)

# Number of lines detected
uint32 line_count

# Field features detected
bool center_circle_detected
float64 center_circle_x            # Pusat lingkaran tengah (meter)
float64 center_circle_y
float64 center_circle_radius       # Radius yang terdeteksi

# Corner detections
uint8 corner_count
float64[] corner_positions         # x,y untuk setiap corner

# Penalty area lines
bool penalty_area_own_detected
bool penalty_area_opp_detected

# Intersection points (berguna untuk lokalisasi)
uint8 intersection_count
float64[] intersection_positions   # x,y untuk setiap intersection

# Line types (jika dapat diklasifikasi)
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_SIDELINE = 1
uint8 TYPE_GOALLINE = 2
uint8 TYPE_CENTERLINE = 3
uint8 TYPE_PENALTY_AREA = 4
uint8 TYPE_GOAL_AREA = 5
uint8[] line_types                 # Type untuk setiap line segment

# Confidence
float64 confidence                 # Overall confidence

# Processing info
uint8 camera_source                # 0=OMNI, 1=FRONT
float32 processing_time_ms
