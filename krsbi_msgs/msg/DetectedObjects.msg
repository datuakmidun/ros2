# DetectedObjects.msg
# Kumpulan semua objek yang terdeteksi dalam satu frame

builtin_interfaces/Time stamp

# Header info
uint8 camera_source                # Sumber kamera (OMNI=0, FRONT=1, FUSED=2)
uint32 frame_id                    # ID frame untuk synchronization

# All detected objects
krsbi_msgs/DetectedObject[] objects

# Quick access counts
uint8 ball_count                   # Jumlah bola terdeteksi
uint8 teammate_count               # Jumlah robot teman terdeteksi
uint8 opponent_count               # Jumlah robot lawan terdeteksi
uint8 goalpost_count               # Jumlah tiang gawang terdeteksi

# Best ball (confidence tertinggi)
krsbi_msgs/DetectedObject best_ball
bool has_ball                      # Ada bola terdeteksi

# Processing info
float32 processing_time_ms         # Waktu processing (milliseconds)
