set(config_module_list_external
	modules/vehicle_models
)

if(NOT PX4_PLATFORM STREQUAL "nuttx")
	list(APPEND config_module_list_external
		modules/control_mixer
		modules/guidance
		modules/flight_modes
		modules/simulation/tv3_sih
	)
endif()

set(config_module_list_external
	${config_module_list_external}
	PARENT_SCOPE
)
