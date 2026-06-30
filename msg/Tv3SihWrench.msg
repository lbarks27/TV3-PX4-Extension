
uint64 timestamp

bool on_rail
float32 rail_torque_scale

# Net engine thrust force in body frame (N).
float32 body_force_n_x
float32 body_force_n_y
float32 body_force_n_z
# r x F torque from engine thrusts before gyroscopic/damping terms (Nm).
float32 engine_torque_nm_x
float32 engine_torque_nm_y
float32 engine_torque_nm_z
# Torque applied to the attitude integrator after rail scaling and rate damping (Nm).
float32 net_torque_nm_x
float32 net_torque_nm_y
float32 net_torque_nm_z
