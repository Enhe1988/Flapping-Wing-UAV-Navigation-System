# ID250 DEBUG_VECT application contract: x/y are velocities; z is height.
# header.frame_id identifies the two-axis input, NOT a complete 3D velocity.
std_msgs/Header header
uint8 system_id
uint8 component_id
uint64 sample_time_us
string name
bool velocity_valid
bool height_valid
float64 forward_velocity_mps
float64 right_velocity_mps
float64 height_up_m
string reason
