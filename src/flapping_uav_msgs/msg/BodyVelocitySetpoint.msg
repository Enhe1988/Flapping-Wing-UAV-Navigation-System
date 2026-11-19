# Publication time. Generic 3D: base_link_frd; agreed planar profile:
# base_link_heading_frd (horizontal forward/right, yaw-only rotation, z=0).
std_msgs/Header header
bool valid
string reason
# Monotonic ages at publication; not refreshed by republishing old inputs.
float64 command_age_ms
float64 attitude_age_ms
geometry_msgs/Vector3 velocity
