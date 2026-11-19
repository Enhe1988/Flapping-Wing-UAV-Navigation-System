#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace flapping_uav_gcs
{
inline double unknown() {return std::numeric_limits<double>::quiet_NaN();}

enum class UwbState : uint8_t
{
  no_data, valid, stream_timeout, target_missing, nonfinite_position, position_jump,
  coordinates_unconfirmed, duplicate_stale, ambiguous_target
};
const char * state_name(UwbState state);

struct UwbConfig
{
  int tag_id{0};
  int tag_role{-1};  // -1: no additional role filter; never guess a numeric Tag role.
  double timeout_s{0.5};
  bool coordinates_confirmed{false};
  double position_scale{1.0};
  // Raw UWB XY may be mirrored: +Y is physically right of +X on this layout.
  // Reflect Y before the proper yaw/translation into right-handed ROS map.
  double y_sign{1.0};
  double yaw_rad{0.0};
  std::array<double, 3> translation{{0.0, 0.0, 0.0}};
  double max_jump_m{0.0};  // 0 disables this uncalibrated threshold.
  bool collect_static_noise{false};
  unsigned noise_sample_limit{100};
  double measured_stddev_xy_m{-1.0};  // negative: no measured covariance supplied.
};

struct UwbTag
{
  uint8_t role{0};
  uint8_t id{0};
  std::array<double, 3> position{{unknown(), unknown(), unknown()}};
};
struct UwbFrame
{
  uint8_t role{0};
  uint8_t anchor_id{0};
  uint32_t local_time{0};
  uint32_t system_time{0};
  std::vector<UwbTag> tags;
};
struct UwbSnapshot
{
  UwbState state{UwbState::no_data};
  uint8_t anchor_id{0};
  bool position_valid{false};
  bool covariance_valid{false};
  std::array<double, 3> raw_position{{unknown(), unknown(), unknown()}};
  std::array<double, 3> map_position{{unknown(), unknown(), unknown()}};
  std::array<double, 4> covariance_xy{{unknown(), unknown(), unknown(), unknown()}};
  double frame_age_s{unknown()};
  double unique_frame_age_s{unknown()};
  double position_age_s{unknown()};
  double accepted_rate_hz{0.0};
  uint64_t frames_received{0}, accepted_positions{0}, rejected_frames{0};
  uint64_t duplicate_frames{0}, jump_rejections{0}, device_time_backwards{0};
  unsigned static_noise_samples{0};
};

class UwbProcessor
{
public:
  explicit UwbProcessor(UwbConfig config);
  // True means exactly one new position may be published; status is separate.
  bool process(const UwbFrame & frame, double received_monotonic_s);
  UwbSnapshot snapshot(double now_monotonic_s) const;
  void reset_observation();
  const UwbConfig & config() const {return config_;}

private:
  void reject(UwbState state);
  void collect_noise(const std::array<double, 3> & position);
  UwbConfig config_;
  UwbSnapshot data_;
  std::optional<double> last_frame_, last_unique_, last_position_;
  std::optional<std::array<uint32_t, 4>> last_key_;
  std::deque<double> recent_positions_;
  std::array<double, 2> mean_{{0.0, 0.0}};
  double m2_x_{0.0}, m2_y_{0.0}, m2_xy_{0.0};
};
}  // namespace flapping_uav_gcs
