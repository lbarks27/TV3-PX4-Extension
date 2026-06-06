project(tv3_rocket_gz)

add_library(${PROJECT_NAME} SHARED
	Tv3RocketSystem.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
	OUTPUT_NAME tv3_rocket_gz
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
	PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
	PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME}
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
	PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
