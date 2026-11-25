# PC position-to-attitude PID diagnostics. Map X/Y axes, NOT FC pitch/roll.
std_msgs/Header header
bool valid
geometry_msgs/Vector3 p_map_deg
geometry_msgs/Vector3 i_map_deg
geometry_msgs/Vector3 d_map_deg
# Causal estimate from filtered UWB positions; never optical-flow telemetry.
geometry_msgs/Vector3 velocity_map_mps
bool velocity_valid
bool integrating
bool limited
