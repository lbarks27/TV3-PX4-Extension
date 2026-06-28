uint64 timestamp

bool on_rail
float32 rail_torque_scale

# Net engine thrust force in body frame (N).
float32[3] body_force_n
# r x F torque from engine thrusts before gyroscopic/damping terms (Nm).
float32[3] engine_torque_nm
# Torque applied to the attitude integrator after rail scaling and rate damping (Nm).
float32[3] net_torque_nm