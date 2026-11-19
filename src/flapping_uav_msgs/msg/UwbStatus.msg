uint8 NO_DATA=0
uint8 VALID=1
uint8 STREAM_TIMEOUT=2
uint8 TARGET_MISSING=3
uint8 NONFINITE_POSITION=4
uint8 POSITION_JUMP=5
uint8 COORDINATES_UNCONFIRMED=6
uint8 DUPLICATE_STALE=7
uint8 AMBIGUOUS_TARGET=8

# Status publication time; never interpreted as a new position observation.
std_msgs/Header header
uint8 state
string reason
uint8 tag_id
int16 configured_tag_role
uint8 anchor_id
bool position_valid
bool coordinates_confirmed
bool covariance_valid
# Last observed raw Tag coordinates, including NaN when unknown.
geometry_msgs/Point raw_position
# Ages use the ground monotonic clock; NaN means never received.
float64 frame_age_ms
float64 unique_frame_age_ms
float64 position_age_ms
float64 accepted_rate_hz
uint64 frames_received
uint64 accepted_positions
uint64 rejected_frames
uint64 duplicate_frames
uint64 jump_rejections
uint64 device_time_backwards
uint32 static_noise_samples
# Measured/configured XY covariance in map, row-major [xx, xy, yx, yy].
# NaN if covariance has not been established.
float64[4] covariance_xy
float64 stddev_x_m
float64 stddev_y_m
