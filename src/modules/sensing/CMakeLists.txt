px4_add_module(
	MODULE modules__tv3_load_cell
	MAIN tv3_load_cell
	SRCS
		tv3_load_cell.cpp
)

px4_add_module(
	MODULE modules__tv3_load_cell_telemetry
	MAIN tv3_load_cell_telemetry
	SRCS
		tv3_load_cell_telemetry.cpp
)
