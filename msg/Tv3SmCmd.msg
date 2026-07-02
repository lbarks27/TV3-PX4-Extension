
uint64 timestamp

uint32 sequence

uint8 COMMAND_NONE = 0
uint8 COMMAND_LAUNCH = 1
uint8 COMMAND_ABORT = 2
uint8 COMMAND_RESET = 3
uint8 command

uint8 SOURCE_INTERNAL = 0
uint8 SOURCE_GCS = 1
uint8 SOURCE_SCRIPT = 2
uint8 SOURCE_TEST = 3
uint8 source

float32 param1
float32 param2

char[32] selected_motor_id
