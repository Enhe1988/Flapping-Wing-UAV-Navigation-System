#include "flapping_uav_gcs/uwb_processor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace flapping_uav_gcs
{
const char * state_name(UwbState state)
{
  switch (state) {
    case UwbState::no_data: return "No Anchor_Frame0 data received";
    case UwbState::valid: return "Fresh Tag XY in map";
    case UwbState::stream_timeout: return "Anchor stream timed out";
    case UwbState::target_missing: return "Configured Tag absent from nodes";
    case UwbState::nonfinite_position: return "Tag XY is not finite";
    case UwbState::position_jump: return "Tag XY jump rejected; inspect data or reset observation";
    case UwbState::coordinates_unconfirmed: return "Confirm units and UWB-to-map calibration";
    case UwbState::duplicate_stale: return "Repeated device timestamps; no fresh observation";
    case UwbState::ambiguous_target: return "Multiple nodes match the configured Tag";
  }
  return "Unknown state";
}

UwbProcessor::UwbProcessor(UwbConfig config)
: config_(std::move(config))
{
  if (config_.tag_id < 0 || config_.tag_id > 255 || config_.tag_role < -1 ||
    config_.tag_role > 255 || !std::isfinite(config_.timeout_s) || config_.timeout_s <= 0 ||
    !std::isfinite(config_.position_scale) || config_.position_scale <= 0 ||
    !std::isfinite(config_.y_sign) || std::abs(config_.y_sign) != 1.0 ||
    !std::isfinite(config_.yaw_rad) || !std::isfinite(config_.max_jump_m) ||
    config_.max_jump_m < 0 || config_.noise_sample_limit < 2 ||
    !std::isfinite(config_.measured_stddev_xy_m) || config_.measured_stddev_xy_m == 0 ||
    !std::all_of(
      config_.translation.begin(), config_.translation.end(),
      [](double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("Invalid UWB parameters (Tag, timeout, transform or noise settings)");
  }
}

void UwbProcessor::reject(UwbState state)
{
  data_.state = state;
  ++data_.rejected_frames;
}

bool UwbProcessor::process(const UwbFrame & frame, double now)
{
  ++data_.frames_received;
  // A new stream session may restart its device clock at the same value.
  if (last_frame_ && now - *last_frame_ > config_.timeout_s) {
    last_key_.reset();
  }
  last_frame_ = now;
  data_.anchor_id = frame.anchor_id;
  const std::array<uint32_t, 4> key{{frame.role, frame.anchor_id,
    frame.local_time, frame.system_time}};
  if (last_key_ && *last_key_ == key) {
    ++data_.duplicate_frames;
    return false;
  }
  if (last_key_ && (*last_key_)[0] == key[0] && (*last_key_)[1] == key[1]) {
    const uint32_t delta = frame.local_time - (*last_key_)[2];
    if (delta > 0x80000000U) {++data_.device_time_backwards;}
  }
  last_key_ = key;
  last_unique_ = now;
  const UwbTag * tag = nullptr;
  for (const auto & candidate : frame.tags) {
    if (candidate.id == config_.tag_id &&
      (config_.tag_role < 0 || candidate.role == config_.tag_role))
    {
      if (tag) {reject(UwbState::ambiguous_target); return false;}
      tag = &candidate;
    }
  }
  if (!tag) {reject(UwbState::target_missing); return false;}
  data_.raw_position = tag->position;
  if (!std::isfinite(tag->position[0]) || !std::isfinite(tag->position[1])) {
    reject(UwbState::nonfinite_position); return false;
  }
  if (!config_.coordinates_confirmed) {
    reject(UwbState::coordinates_unconfirmed); return false;
  }
  const double x = tag->position[0] * config_.position_scale;
  const double y = tag->position[1] * config_.position_scale * config_.y_sign;
  const double c = std::cos(config_.yaw_rad), s = std::sin(config_.yaw_rad);
  std::array<double, 3> mapped{{c * x - s * y + config_.translation[0],
    s * x + c * y + config_.translation[1],
    tag->position[2] * config_.position_scale + config_.translation[2]}};
  if (!std::isfinite(mapped[2])) {mapped[2] = unknown();}
  if (!std::isfinite(mapped[0]) || !std::isfinite(mapped[1])) {
    reject(UwbState::nonfinite_position); return false;
  }
  if (last_position_ && config_.max_jump_m > 0 &&
    std::hypot(mapped[0] - data_.map_position[0], mapped[1] - data_.map_position[1]) >
    config_.max_jump_m)
  {
    ++data_.jump_rejections;
    reject(UwbState::position_jump); return false;
  }
  data_.map_position = mapped;
  data_.state = UwbState::valid;
  last_position_ = now;
  ++data_.accepted_positions;
  recent_positions_.push_back(now);
  while (!recent_positions_.empty() &&
    (recent_positions_.size() > 256 || now - recent_positions_.front() > 2.0))
  {
    recent_positions_.pop_front();
  }
  collect_noise(mapped);
  return true;
}

void UwbProcessor::collect_noise(const std::array<double, 3> & p)
{
  if (!config_.collect_static_noise || data_.static_noise_samples >= config_.noise_sample_limit) {
    return;
  }
  const unsigned n = ++data_.static_noise_samples;
  const double dx = p[0] - mean_[0], dy = p[1] - mean_[1];
  mean_[0] += dx / n;
  mean_[1] += dy / n;
  m2_x_ += dx * (p[0] - mean_[0]);
  m2_y_ += dy * (p[1] - mean_[1]);
  m2_xy_ += dx * (p[1] - mean_[1]);
}

UwbSnapshot UwbProcessor::snapshot(double now) const
{
  auto result = data_;
  if (last_frame_) {result.frame_age_s = std::max(0.0, now - *last_frame_);}
  if (last_unique_) {result.unique_frame_age_s = std::max(0.0, now - *last_unique_);}
  if (last_position_) {result.position_age_s = std::max(0.0, now - *last_position_);}
  if (!last_frame_) {
    result.state = UwbState::no_data;
  } else if (result.frame_age_s > config_.timeout_s) {
    result.state = UwbState::stream_timeout;
  } else if (result.unique_frame_age_s > config_.timeout_s) {
    result.state = UwbState::duplicate_stale;
  }
  result.position_valid = result.state == UwbState::valid;
  std::deque<double> recent;
  for (double when : recent_positions_) {
    if (now - when <= 2.0) {
      recent.push_back(when);
    }
  }
  if (result.position_valid && recent.size() >= 2 && recent.back() > recent.front()) {
    result.accepted_rate_hz = (recent.size() - 1) / (recent.back() - recent.front());
  }
  if (config_.measured_stddev_xy_m > 0) {
    const double v = config_.measured_stddev_xy_m * config_.measured_stddev_xy_m;
    result.covariance_xy = {{v, 0.0, 0.0, v}};
    result.covariance_valid = std::isfinite(v);
  } else if (data_.static_noise_samples >= config_.noise_sample_limit) {
    const double denominator = data_.static_noise_samples - 1;
    result.covariance_xy = {{m2_x_ / denominator, m2_xy_ / denominator,
      m2_xy_ / denominator, m2_y_ / denominator}};
    result.covariance_valid = result.covariance_xy[0] > 0 && result.covariance_xy[3] > 0 &&
      std::all_of(
      result.covariance_xy.begin(), result.covariance_xy.end(),
      [](double value) {return std::isfinite(value);});
  }
  if (!result.covariance_valid) {
    result.covariance_xy.fill(unknown());
  }
  return result;
}

void UwbProcessor::reset_observation()
{
  const auto frames = data_.frames_received, accepted = data_.accepted_positions;
  const auto rejected = data_.rejected_frames, duplicate = data_.duplicate_frames;
  const auto jumps = data_.jump_rejections, backwards = data_.device_time_backwards;
  data_ = UwbSnapshot{};
  data_.frames_received = frames; data_.accepted_positions = accepted;
  data_.rejected_frames = rejected; data_.duplicate_frames = duplicate;
  data_.jump_rejections = jumps; data_.device_time_backwards = backwards;
  last_frame_.reset(); last_unique_.reset(); last_position_.reset(); last_key_.reset();
  recent_positions_.clear(); mean_.fill(0.0); m2_x_ = m2_y_ = m2_xy_ = 0.0;
}
}  // namespace flapping_uav_gcs
