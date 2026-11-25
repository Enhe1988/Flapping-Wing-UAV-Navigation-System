# Publication time. Measurement/prediction freshness is position_age_ms, not this header alone.
std_msgs/Header header
bool valid
string source
string reason
geometry_msgs/Point position
geometry_msgs/Point raw_uwb_position
geometry_msgs/Point optical_odometry_position
bool uwb_healthy
bool flow_valid
float64 position_age_ms
float64 reliable_uwb_age_ms
float64 raw_uwb_age_ms
float64 flow_age_ms
float64 innovation_m
int64 alignment_reference_ns
uint64 source_switches
uint64 uwb_syncs
