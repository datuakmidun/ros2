# BallPosition.msg
# Position dan status bola yang terdeteksi oleh sistem vision
# Koordinat dalam frame robot (robot-centric)

builtin_interfaces/Time stamp      # Timestamp deteksi

# Posisi bola relatif terhadap robot
float64 x                          # Jarak ke depan (meter), positif = depan robot
float64 y                          # Jarak ke samping (meter), positif = kiri robot
float64 z                          # Tinggi bola (meter), untuk deteksi bola melayang

# Posisi dalam koordinat polar (untuk kemudahan)
float64 distance                   # Jarak langsung ke bola (meter)
float64 angle                      # Sudut ke bola (radian), 0 = depan, positif = kiri

# Confidence dan status
float64 confidence                 # Tingkat kepercayaan deteksi (0.0 - 1.0)
bool is_visible                    # Apakah bola terlihat saat ini
bool is_moving                     # Apakah bola sedang bergerak

# Velocity estimation (jika tracking aktif)
float64 vx                         # Kecepatan bola arah x (m/s)
float64 vy                         # Kecepatan bola arah y (m/s)

# Source camera
uint8 CAMERA_OMNI = 0
uint8 CAMERA_FRONT = 1
uint8 CAMERA_FUSED = 2
uint8 camera_source                # Sumber deteksi
