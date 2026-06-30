px4_add_module(
	MODULE modules__tv3_guidance
	MAIN tv3_guidance
	SRCS
		tv3_guidance.cpp
		tv3_guidance_fsm.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../..
)
