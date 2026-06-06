uint64 timestamp

bool loaded
bool active

float32 expected_thrust_n
float32 expected_motor_mass_kg
float32 expected_vehicle_mass_kg
float32 total_impulse_ns
float32 burn_fraction
float32 burn_time_s
float32 burn_duration_s

uint16 selected_motor_index
char[32] selected_motor_id

uint8 engine_count
uint8 ignition_mask
uint8 active_mask
float32[4] expected_thrust_n_engine
float32[4] expected_motor_mass_kg_engine
float32[4] burn_fraction_engine
float32[4] burn_time_s_engine
uint16[4] selected_motor_index_engine
