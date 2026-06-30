
uint64 timestamp
uint64 timestamp_sample

int8 channel
int32 raw_count
float32 voltage_v
float32 thrust_n

bool valid

uint8 FAULT_NONE = 0
uint8 FAULT_STALE = 1
uint8 FAULT_CHANNEL_MISSING = 2
uint8 FAULT_BAD_SCALE = 4
uint8 FAULT_NO_SAMPLE = 8
uint8 fault_flags
