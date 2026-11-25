# Aircraft controller convention: forward tilt/pitch positive, left roll positive.
# Horizontal attitude targets only; yaw and thrust remain on the FC.
std_msgs/Header header
bool valid
string reason
float64 command_age_ms
float64 attitude_age_ms
float64 pitch_deg
float64 roll_deg
