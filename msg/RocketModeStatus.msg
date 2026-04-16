uint64 timestamp

uint8 STATE_DISARMED_SAFE = 0
uint8 STATE_ARMED_STANDBY = 1
uint8 STATE_READY = 2
uint8 STATE_IGNITION_PENDING = 3
uint8 STATE_BOOST = 4
uint8 STATE_COAST = 5
uint8 STATE_ABORT = 6
uint8 state

bool mode_active
bool armed
bool ignition_on
bool load_cell_valid
bool gcs_link_ok

float32 thrust_n
float32 burn_time_s
float32 motor_mass_kg
float32 vehicle_mass_kg
float32 rail_distance_m

uint32 ABORT_REASON_NONE = 0
uint32 ABORT_REASON_COMMAND = 1
uint32 ABORT_REASON_IGNITION_TIMEOUT = 2
uint32 ABORT_REASON_SENSOR_STALE = 3
uint32 ABORT_REASON_GCS_LOSS = 4
uint32 abort_reason
