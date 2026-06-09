px4_add_module(
	MODULE modules__simulation__tv3_sih
	MAIN tv3_sih
	SRCS
		tv3_sih.cpp
)

if(PX4_PLATFORM MATCHES "posix")
	add_custom_target(sihsim_tv3_lander
		COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=tv3_lander PX4_SIMULATOR=sihsim $<TARGET_FILE:px4>
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		DEPENDS px4
	)
endif()
