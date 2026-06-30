px4_add_module(
	MODULE modules__simulation__tv3_sih
	MAIN tv3_sih
	SRCS
		tv3_sih.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../../..
)

if(PX4_PLATFORM MATCHES "posix")
	add_custom_target(sihsim_tv3_lander
		COMMAND ${CMAKE_COMMAND} -E env
			PX4_SIM_MODEL=tv3_lander
			PX4_SIMULATOR=sihsim
			TV3_MOTOR_ROOT=$ENV{TV3_MOTOR_ROOT}
			$<TARGET_FILE:px4> -s etc/init.d-posix/rcS 0
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		DEPENDS px4
	)
endif()
