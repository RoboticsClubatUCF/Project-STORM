# Teleop launcher state, published at 1 Hz on /storm/teleop/status

bool running
string profile     # last started profile, empty if never started
bool use_joy
float64 uptime_s   # s since start, 0 when stopped
string message     # last event, e.g. "stopped" or "exited with code 1"
