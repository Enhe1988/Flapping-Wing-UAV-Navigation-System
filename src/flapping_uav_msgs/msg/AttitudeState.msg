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
