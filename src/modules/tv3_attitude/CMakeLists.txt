px4_add_module(
	MODULE modules__tv3_attitude
	MAIN tv3_attitude
	SRCS
		tv3_attitude.cpp
		tv3_attitude_fsm.cpp
		tv3_attitude_controller.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../..
)
