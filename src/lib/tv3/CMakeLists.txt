px4_add_library(tv3_control
	tv3_plant_kinematics.cpp
	tv3_projected_gradient.cpp
)

target_include_directories(tv3_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
