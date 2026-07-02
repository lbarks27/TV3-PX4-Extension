px4_add_module(
	MODULE modules__tv3_params
	MAIN tv3_params
	SRCS
		tv3_params.cpp
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../..
	EXTERNAL
)
