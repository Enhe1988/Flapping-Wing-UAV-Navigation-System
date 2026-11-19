#ifndef FLAPPING_UAV_GCS__ALIGNMENT_SESSION_HPP_
#define FLAPPING_UAV_GCS__ALIGNMENT_SESSION_HPP_
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <time.h>
#include "rclcpp/rclcpp.hpp"

namespace flapping_uav_gcs
{
struct AlignmentSessionConfig
{
  bool required{false};
  int64_t device_time_us{0}, host_monotonic_ns{0};
  std::string host_boot_id;
  double max_clock_skew_ms{1000};
};
inline bool alignment_matches_session(
  const AlignmentSessionConfig & c, const std::string & host_boot_id,
  uint64_t sample_time_us, int64_t now_monotonic_ns)
{
  if (!c.required) {return true;}
  if (c.device_time_us <= 0 || c.host_monotonic_ns <= 0 || c.host_boot_id.empty() ||
    host_boot_id != c.host_boot_id || now_monotonic_ns < c.host_monotonic_ns ||
    sample_time_us < static_cast<uint64_t>(c.device_time_us) ||
    !std::isfinite(c.max_clock_skew_ms) || c.max_clock_skew_ms <= 0) {return false;}
  const long double host_elapsed_ms =
    static_cast<long double>(now_monotonic_ns - c.host_monotonic_ns) / 1e6L;
  const long double device_elapsed_ms =
    (static_cast<long double>(sample_time_us) - c.device_time_us) / 1000.L;
  return std::abs(host_elapsed_ms - device_elapsed_ms) <= c.max_clock_skew_ms;
}
class AlignmentSession
{
public:
  void load(rclcpp::Node & node)
  {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    config_.required=node.declare_parameter<bool>("alignment_requires_session",false,d);
    config_.device_time_us=node.declare_parameter<int64_t>("alignment_device_time_us",0,d);
    config_.host_monotonic_ns=node.declare_parameter<int64_t>("alignment_host_monotonic_ns",0,d);
    config_.host_boot_id=node.declare_parameter<std::string>("alignment_host_boot_id","",d);
    config_.max_clock_skew_ms=node.declare_parameter<double>("alignment_max_clock_skew_ms",1000.,d);
    std::ifstream file("/proc/sys/kernel/random/boot_id");std::getline(file,host_boot_id_);
  }
  const AlignmentSessionConfig & reference() const {return config_;}
  void set_reference(const AlignmentSessionConfig & reference) {config_ = reference;}
  bool bind(uint64_t sample_time_us)
  {
    timespec t{};
    if (sample_time_us == 0 || host_boot_id_.empty() ||
      ::clock_gettime(CLOCK_MONOTONIC, &t) != 0) {return false;}
    config_.required = true;
    config_.device_time_us = static_cast<int64_t>(sample_time_us);
    config_.host_monotonic_ns = static_cast<int64_t>(t.tv_sec)*1000000000LL+t.tv_nsec;
    config_.host_boot_id = host_boot_id_;
    return true;
  }
  bool valid(uint64_t sample_time_us) const
  {
    timespec t{};
    if (::clock_gettime(CLOCK_MONOTONIC,&t)!=0) {return !config_.required;}
    const int64_t now_ns=static_cast<int64_t>(t.tv_sec)*1000000000LL+t.tv_nsec;
    return alignment_matches_session(config_,host_boot_id_,sample_time_us,now_ns);
  }
private:
  AlignmentSessionConfig config_;
  std::string host_boot_id_;
};
}  // namespace flapping_uav_gcs
#endif
