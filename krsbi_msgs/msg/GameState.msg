# GameState.msg
# Status permainan sesuai aturan KRSBI-B / RoboCup

builtin_interfaces/Time stamp

# Game phases (sesuai GameController)
uint8 PHASE_INITIAL = 0            # Belum dimulai
uint8 PHASE_READY = 1              # Persiapan, robot ke posisi awal
uint8 PHASE_SET = 2                # Set, menunggu kick-off
uint8 PHASE_PLAYING = 3            # Permainan berlangsung
uint8 PHASE_FINISHED = 4           # Babak/pertandingan selesai
uint8 phase

# Secondary state
uint8 STATE_NORMAL = 0
uint8 STATE_PENALTYSHOOT = 1       # Penalty shootout
uint8 STATE_OVERTIME = 2           # Overtime
uint8 STATE_TIMEOUT = 3            # Timeout
uint8 secondary_state

# Set play types
uint8 SETPLAY_NONE = 0
uint8 SETPLAY_KICKOFF_OWN = 1      # Kick-off tim sendiri
uint8 SETPLAY_KICKOFF_OPP = 2      # Kick-off lawan
uint8 SETPLAY_FREEKICK_OWN = 3     # Free kick tim sendiri
uint8 SETPLAY_FREEKICK_OPP = 4     # Free kick lawan
uint8 SETPLAY_GOALKICK_OWN = 5     # Goal kick tim sendiri
uint8 SETPLAY_GOALKICK_OPP = 6     # Goal kick lawan
uint8 SETPLAY_CORNER_OWN = 7       # Corner kick tim sendiri
uint8 SETPLAY_CORNER_OPP = 8       # Corner kick lawan
uint8 SETPLAY_THROWIN_OWN = 9      # Throw-in tim sendiri
uint8 SETPLAY_THROWIN_OPP = 10     # Throw-in lawan
uint8 SETPLAY_PENALTY_OWN = 11     # Penalty tim sendiri
uint8 SETPLAY_PENALTY_OPP = 12     # Penalty lawan
uint8 set_play

# Team info
uint8 team_color                   # 0 = Blue, 1 = Yellow
bool is_kickoff_team               # Apakah tim yang kick-off

# Score
uint8 own_score                    # Skor tim sendiri
uint8 opponent_score               # Skor lawan

# Time
int32 time_remaining               # Sisa waktu babak (detik)
uint8 half                         # Babak ke (1 atau 2)

# Penalties (untuk robot ini)
bool is_penalized                  # Robot sedang di-penalti
int32 penalty_time_remaining       # Sisa waktu penalti (detik)

# Ball
bool ball_in_play                  # Bola dalam permainan
