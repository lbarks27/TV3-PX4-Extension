
uint64 timestamp
uint64 timestamp_sample

float32 measured_thrust_n
float32 filtered_thrust_n
float32 expected_thrust_n
float32 expected_motor_mass_kg
float32 expected_vehicle_mass_kg
float32 total_impulse_ns
float32 burn_fraction

bool valid
bool ignition_confirmed

uint8 FAULT_NONE = 0
uint8 FAULT_STALE = 1
uint8 FAULT_CHANNEL_MISSING = 2
uint8 FAULT_BAD_SCALE = 4
uint8 FAULT_NO_REFERENCE = 8
uint8 fault_flags

uint16 selected_motor_index
char[32] selected_motor_id
