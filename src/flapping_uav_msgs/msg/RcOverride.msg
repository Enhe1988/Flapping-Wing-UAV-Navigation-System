# Publication time; input freshness is reported separately using a monotonic clock.
std_msgs/Header header
bool valid
bool device_connected
string reason
float64 input_age_ms
uint64 input_frames
uint16[18] channels
