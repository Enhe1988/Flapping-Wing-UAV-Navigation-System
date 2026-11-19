# Independent XY observer status. Never used as navigation input by default.
std_msgs/Header header
string state
string reason
bool initialized
bool valid
bool uwb_recent
bool flow_valid
bool alignment_valid
bool propagation_complete
# Covariance is an observation-model estimate, not a calibrated accuracy guarantee.
bool covariance_is_model
geometry_msgs/Point position
geometry_msgs/Vector3 velocity_map
float64 uwb_age_ms
float64 flow_age_ms
float64 attitude_age_ms
float64 position_age_ms
float64 innovation_m
float64 attitude_pair_skew_ms
float64 map_to_enu_yaw_rad
uint64 accepted_uwb
uint64 rejected_uwb
uint64 duplicate_uwb
uint64 duplicate_flow
uint64 flow_gaps
