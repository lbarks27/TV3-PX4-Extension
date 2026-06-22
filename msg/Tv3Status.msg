uint64 timestamp

uint8 MODE_DISARMED_SAFE = 0
uint8 MODE_ARMED_STANDBY = 1
uint8 MODE_READY = 2
uint8 MODE_IGNITION_PENDING = 3
uint8 MODE_BOOST = 4
uint8 MODE_COAST = 5
uint8 MODE_ABORT = 6
uint8 mode

bool mode_active
bool armed
bool ready
bool ignition_on
bool ignition_confirmed
bool rail_exit
bool burnout_detected
bool thrust_valid
bool gcs_link_ok

uint64 ignition_timestamp
uint64 boost_timestamp

float32 measured_thrust_n
float32 filtered_thrust_n
float32 expected_thrust_n
float32 burn_time_s
float32 rail_distance_m
float32 rail_velocity_m_s
float32 expected_motor_mass_kg
float32 expected_vehicle_mass_kg
float32 burn_fraction

uint32 FAULT_NONE = 0
uint32 FAULT_COMMAND_ABORT = 1
uint32 FAULT_IGNITION_TIMEOUT = 2
uint32 FAULT_SENSOR_STALE = 4
uint32 FAULT_GCS_LOSS = 8
uint32 FAULT_MOTOR_DATA = 16
uint32 FAULT_ARMING = 32
uint32 fault_reason

uint16 selected_motor_index
char[32] selected_motor_id

uint8 engine_count
uint8 ignition_mask
uint8 active_ignition_index
bool sequence_active
bool sequence_complete
