px4_add_module(
	MODULE modules__tv3_control_mixer
	MAIN tv3_control_mixer
	SRCS
		tv3_control_mixer.cpp
		tv3_gimbal_authority.cpp
		tv3_gimbal_plant.cpp
		tv3_gimbal_lm.cpp
)
