px4_add_module(
	MODULE modules__rocket_motor_model
	MAIN rocket_motor_model
	SRCS
		rocket_motor_model.cpp
)

px4_add_module(
	MODULE modules__rocket_load_cell
	MAIN rocket_load_cell
	SRCS
		rocket_load_cell.cpp
)
