uint64 timestamp
uint64 timestamp_sample

uint8 MAX_ENGINES = 4
uint8 engine_count
uint8 ignition_mask
uint8 active_mask
uint8 confirmed_mask
uint8 fault_mask

bool sequence_complete
bool all_ignited

float32[4] measured_thrust_n
float32[4] filtered_thrust_n
float32[4] expected_thrust_n
float32[4] expected_motor_mass_kg
float32[4] burn_fraction
float32[4] remaining_impulse_ns

uint8[4] fault_flags
uint16[4] selected_motor_index
