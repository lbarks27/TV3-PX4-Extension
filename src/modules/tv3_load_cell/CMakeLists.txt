px4_add_module(
	MODULE modules__tv3_load_cell
	MAIN tv3_load_cell
	SRCS
		tv3_load_cell.cpp
		tv3_load_cell_fsm.cpp
		../../lib/tv3_motor_curve.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../..
)
