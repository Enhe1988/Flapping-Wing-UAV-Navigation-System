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
# Actual position input used by the controller, including its validity.
bool optical_flow_fallback_enabled
bool position_valid
string position_source
geometry_msgs/Point position_used
# Analytic horizontal reference, distinct from the velocity-loop command above.
string flight_mode
string trajectory_phase
bool trajectory_running
float64 trajectory_time_s
float64 trajectory_period_s
geometry_msgs/Vector3 trajectory_velocity_map
# velocity or attitude; velocity_map is NaN when the output is an angle.
string horizontal_control_mode
float64 target_pitch_deg
float64 target_roll_deg
