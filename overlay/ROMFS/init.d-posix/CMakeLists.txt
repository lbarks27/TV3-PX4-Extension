add_subdirectory(airframes)

px4_add_romfs_files(
	px4-rc.mavlink
	px4-rc.simulator
	px4-rc.jmavsim
	px4-rc.mavlinksim
	px4-rc.sihsim
	rc.tv3_defaults
	rc.replay
	rcS
)
