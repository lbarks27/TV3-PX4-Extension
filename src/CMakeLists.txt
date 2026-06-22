# PX4 includes px4_add_library only after processing external modules, so make
# the helper available here before building the shared tv3 control library.
include(px4_add_library)

add_subdirectory(lib/tv3)

set(config_module_list_external
	modules/sensing
	modules/propulsion
)

if(NOT PX4_PLATFORM STREQUAL "nuttx")
	list(APPEND config_module_list_external
		modules/vehicle
		modules/guidance
		modules/control/reference
		modules/control/attitude
		modules/control/allocation
		modules/simulation/tv3_sih
	)
endif()

set(config_module_list_external
	${config_module_list_external}
	PARENT_SCOPE
)
