px4_add_module(
	MODULE modules__tv3_tvc_allocator
	MAIN tv3_tvc_allocator
	SRCS
		tv3_tvc_allocator.cpp
	DEPENDS
		tv3_control
)
