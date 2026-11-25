#ifndef FLAPPING_UAV_GCS__SAMPLE_ORDER_HPP_
#define FLAPPING_UAV_GCS__SAMPLE_ORDER_HPP_

#include <cstdint>

namespace flapping_uav_gcs
{
enum class SampleOrder {accepted, duplicate, old_packet, future_packet, clock_reset};

// UDP may deliver an isolated older sample. It must not refresh input age or
// destroy a calibrated reference. Confirm a new clock epoch with at least three
// advancing lower timestamps over >=50ms. An explicit reset counter is immediate.
class SampleOrderFilter
{
public:
  SampleOrder observe(uint64_t device_us, uint8_t reset_counter, int64_t host_ns)
  {
    if (!seen_) {
      accept(device_us, reset_counter, host_ns); return SampleOrder::accepted;
    }
    if (reset_counter != reset_counter_) {
      accept(device_us, reset_counter, host_ns); return SampleOrder::clock_reset;
    }
    if (device_us > last_us_) {
      // An implausible forward sample must not poison the accepted epoch and
      // make subsequent ordinary samples look like a reboot. Allow 2 seconds
      // of transport catch-up beyond elapsed host time, matching session skew.
      const uint64_t host_us=host_ns>=last_host_ns_?
        static_cast<uint64_t>(host_ns-last_host_ns_)/1000:0;
      const uint64_t advance=device_us-last_us_;
      if(advance>host_us && advance-host_us>2000000ULL){return SampleOrder::future_packet;}
      accept(device_us, reset_counter, host_ns); return SampleOrder::accepted;
    }
    if (device_us == last_us_) {return SampleOrder::duplicate;}
    if (candidate_count_ == 0 || device_us < candidate_us_ || host_ns < candidate_start_ns_) {
      candidate_count_ = 1; candidate_start_ns_ = host_ns;
    } else if (device_us > candidate_us_) {++candidate_count_;}
    candidate_us_ = device_us;
    if (candidate_count_ >= 3 && host_ns - candidate_start_ns_ >= 50000000LL) {
      accept(device_us, reset_counter, host_ns); return SampleOrder::clock_reset;
    }
    return SampleOrder::old_packet;
  }
  uint64_t last_time_us() const {return last_us_;}
private:
  void accept(uint64_t device_us, uint8_t reset_counter, int64_t host_ns)
  {
    seen_ = true; last_us_ = device_us; reset_counter_ = reset_counter;last_host_ns_=host_ns;
    candidate_count_ = 0;
  }
  bool seen_{false};
  uint64_t last_us_{0}, candidate_us_{0};
  uint8_t reset_counter_{0};
  unsigned candidate_count_{0};
  int64_t candidate_start_ns_{0},last_host_ns_{0};
};
}  // namespace flapping_uav_gcs
#endif
