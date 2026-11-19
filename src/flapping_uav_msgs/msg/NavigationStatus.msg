std_msgs/Header header
string state
string reason
bool enabled
# True when fresh ID250 has invalid XY and configured fallback uses zero.
bool feedback_assumed_zero
bool inputs_valid
bool goal_active
bool reached
geometry_msgs/Point goal
float64 distance_m
float64 actual_speed_mps
float64 position_age_ms
float64 feedback_age_ms
float64 attitude_age_ms
float64 heartbeat_age_ms
geometry_msgs/Vector3 velocity_map
