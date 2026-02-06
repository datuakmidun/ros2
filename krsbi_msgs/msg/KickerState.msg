# KickerState.msg
# Status sistem penendang dengan capacitor dan 2 relay
#
# Sistem kicker:
# - Relay 1: Charging relay (menghubungkan charger ke capacitor)
# - Relay 2: Discharge relay (melepas energi dari capacitor ke solenoid)
#
# Safety: Kedua relay TIDAK BOLEH aktif bersamaan!

builtin_interfaces/Time stamp

# Capacitor status
float32 voltage                    # Tegangan capacitor saat ini (Volt)
float32 target_voltage             # Target tegangan charging (Volt)
float32 charge_percentage          # Persentase charge (0-100%)

# Relay states
bool relay_charge                  # Status relay charging (true = charging)
bool relay_kick                    # Status relay kick (true = discharging/kicking)

# Kicker states
uint8 STATE_IDLE = 0               # Tidak ada aktivitas
uint8 STATE_CHARGING = 1           # Sedang charging capacitor
uint8 STATE_READY = 2              # Siap tendang (fully charged)
uint8 STATE_KICKING = 3            # Sedang menendang
uint8 STATE_COOLDOWN = 4           # Cooldown setelah tendang
uint8 STATE_ERROR = 5              # Error state
uint8 state

# Kick power setting
uint8 POWER_LOW = 1                # Tendangan pelan (pass)
uint8 POWER_MEDIUM = 2             # Tendangan sedang
uint8 POWER_HIGH = 3               # Tendangan keras (shoot)
uint8 power_level

# Timing
float32 charge_time                # Waktu yang dibutuhkan untuk charge penuh (detik)
float32 cooldown_remaining         # Sisa waktu cooldown (detik)

# Safety
bool capacitor_ok                  # Capacitor dalam kondisi baik
bool charger_ok                    # Charger berfungsi
bool safety_ok                     # Semua safety check passed
bool ready_to_kick                 # Siap untuk tendang

# Statistics
uint32 kick_count                  # Jumlah tendangan total
uint32 charge_cycles               # Jumlah siklus charging

# Error
bool error
string error_message
