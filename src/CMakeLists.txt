set(config_module_list_external
	modules/tv3_params
	modules/tv3_load_cell
	modules/tv3_attitude
	modules/tv3_control_mixer
	modules/tv3_guidance
	modules/tv3_state_machine
)

if(NOT PX4_PLATFORM STREQUAL "nuttx")
	list(APPEND config_module_list_external
		modules/simulation/tv3_sih
	)
endif()

set(config_module_list_external
	${config_module_list_external}
	PARENT_SCOPE
)
