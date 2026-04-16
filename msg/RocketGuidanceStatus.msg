uint64 timestamp

uint8 PHASE_STANDBY = 0
uint8 PHASE_LAUNCH_ASCENT = 1
uint8 PHASE_APOGEE_TRACK = 2
uint8 PHASE_WAYPOINT_TRACK = 3
uint8 PHASE_LANDING_APPROACH = 4
uint8 PHASE_COMPLETE = 5
uint8 PHASE_ABORT = 6
uint8 phase

bool active
bool armed
bool origin_valid
bool position_valid
bool rocket_ready
bool rocket_boosting
bool rocket_coasting

uint8 rocket_mode
uint8 waypoint_index
uint8 waypoint_count

float32 target_n
float32 target_e
float32 target_d
float32 target_distance_m
float32 cross_track_error_m
float32 down_track_error_m
float32 commanded_speed_m_s
float32 commanded_climb_rate_m_s
float32 commanded_yaw_rad
