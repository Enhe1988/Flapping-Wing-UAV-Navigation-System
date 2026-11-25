# Reception time; orientation direction/reference remain a commissioning contract.
std_msgs/Header header
string body_frame_id
uint8 system_id
uint8 component_id
uint32 message_id
uint64 sample_time_us
uint8 reset_counter
bool valid
string reason
# ROS field order x,y,z,w; decoded from MAVLink WXYZ without axis swaps.
geometry_msgs/Quaternion orientation
# Unfiltered reported body-axis angular rates, rad/s; not Euler-angle derivatives.
bool angular_velocity_valid
geometry_msgs/Vector3 angular_velocity
# Compact ID254 reports heading only. Quaternion, roll/pitch and body rates
# are unavailable (NaN); yaw_rad follows body-FRD to local-NED convention.
bool yaw_only
float64 yaw_rad
