# Pack voltage as seen by the drivetrain SPARK MAXs

uint8 LEVEL_UNKNOWN=0
uint8 LEVEL_OK=1
uint8 LEVEL_LOW=2
uint8 LEVEL_CRITICAL=3

std_msgs/Header header

float32 voltage        # V, mean of online SPARKs, NaN if none
float32 min_voltage    # V
float32 max_voltage    # V
uint8 sources_online
uint8 sources_total
uint8 level
