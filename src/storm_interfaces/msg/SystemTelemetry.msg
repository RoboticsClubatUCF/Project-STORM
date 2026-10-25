# Onboard computer health. NaN = unavailable.

std_msgs/Header header

float32 cpu_percent        # %, all cores
float32 load_avg_1m
float32 cpu_temp_c         # degC, hottest thermal zone
float32 mem_used_percent   # %
float32 disk_used_percent  # %, root filesystem
float64 uptime_s

string wifi_interface      # empty if no wireless interface
float32 wifi_link_quality  # 0..1
float32 wifi_signal_dbm
