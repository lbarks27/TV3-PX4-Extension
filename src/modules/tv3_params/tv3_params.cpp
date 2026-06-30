#include <px4_platform_common/log.h>

extern "C" __EXPORT int tv3_params_main(int argc, char *argv[]);

int tv3_params_main(int argc, char *argv[])
{
	PX4_INFO("TV3 parameter definitions are compiled into the firmware image");
	return 0;
}
