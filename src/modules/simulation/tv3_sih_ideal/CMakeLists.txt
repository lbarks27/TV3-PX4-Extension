px4_add_module(
	MODULE modules__simulation__tv3_sih_ideal
	MAIN tv3_sih_ideal
	SRCS
		tv3_sih_ideal.cpp
)

if(PX4_PLATFORM MATCHES "posix")
	add_custom_target(sihsim_tv3_lander_ideal
		COMMAND ${CMAKE_COMMAND} -E env
			PX4_SIM_MODEL=tv3_lander
			PX4_SIMULATOR=sihsim
			TV3_SIH_IDEAL=1
			TV3_MOTOR_ROOT=$ENV{TV3_MOTOR_ROOT}
			$<TARGET_FILE:px4> -s etc/init.d-posix/rcS 0
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		DEPENDS px4
	)
endif()