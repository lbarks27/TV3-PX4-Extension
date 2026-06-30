# Shell: PX4 module entry and uORB I/O
# FSM: module mode selection and per-tick orchestration
# Core: geometry, limits, and solve wrapper
# Plant/LM: kinematic wrench model and numerical allocator
px4_add_module(
	MODULE modules__tv3_control_mixer
	MAIN tv3_control_mixer
	SRCS
		tv3_control_mixer.cpp
		tv3_control_mixer_fsm.cpp
		tv3_control_mixer_core.cpp
		tv3_control_mixer_plant.cpp
		tv3_control_mixer_lm.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../..
)
