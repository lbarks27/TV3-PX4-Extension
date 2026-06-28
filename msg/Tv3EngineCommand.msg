uint64 timestamp

uint8 MAX_ENGINES = 4
uint8 engine_count
uint8 ignition_mask
uint8 active_ignition_index

uint16[4] selected_motor_index

bool sequence_active
bool sequence_complete

float32[4] commanded_pitch_deg
float32[4] commanded_yaw_deg
float32[4] commanded_splay_deg
