px4_add_module(
	MODULE modules__tv3_state_machine
	MAIN tv3_state_machine
	SRCS
		tv3_state_machine.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../..
	EXTERNAL
)
