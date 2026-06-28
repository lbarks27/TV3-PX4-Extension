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
float32[3] demanded_torque_nm
# Incremental TVC torque relative to neutral gimbals (excludes thrust-imbalance bias).
float32[3] achieved_torque_nm

# Body-frame thrust force demanded by guidance/LM (N). Axial demand is along body +X.
float32[3] demanded_body_force_n
# Incremental body thrust relative to neutral gimbals (N).
float32[3] achieved_body_force_n
