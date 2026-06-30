
uint64 timestamp

uint8 iterations_used
bool lm_path_active
bool converged
bool demand_saturated
bool used_fallback_solution
bool torque_direction_valid

float32 residual_torque_nm
float32 residual_thrust_n
float32 cost
float32 lambda_final
float32 torque_authority_scale

# Attitude torque demand (matches vehicle_torque_setpoint after limits).
float32 demanded_torque_nm_roll
float32 demanded_torque_nm_pitch
float32 demanded_torque_nm_yaw
# Incremental TVC torque relative to neutral gimbals (excludes thrust-imbalance bias).
float32 achieved_torque_nm_roll
float32 achieved_torque_nm_pitch
float32 achieved_torque_nm_yaw

# Body-frame thrust force demanded by guidance/LM (N). Axial demand is along body +X.
float32 demanded_body_force_n_x
float32 demanded_body_force_n_y
float32 demanded_body_force_n_z
# Incremental body thrust relative to neutral gimbals (N).
float32 achieved_body_force_n_x
float32 achieved_body_force_n_y
float32 achieved_body_force_n_z
