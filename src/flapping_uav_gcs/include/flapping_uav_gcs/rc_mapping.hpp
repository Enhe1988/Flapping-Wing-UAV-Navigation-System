#ifndef FLAPPING_UAV_GCS__RC_MAPPING_HPP_
#define FLAPPING_UAV_GCS__RC_MAPPING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace flapping_uav_gcs
{
struct RcMapping
{
  // User channels 0..3: right X/Y, left Y/X; /joy a2=left Y, a3=left X.
  std::array<int, 8> indices{{0, 1, 2, 3, 4, 5, 6, 7}};
  std::array<double, 8> signs{{-1, -1, -1, -1, -1, -1, -1, -1}};
  std::array<double, 8> centers{{0, 0, 0, 0, 0, 0, 0, 0}};
  // Bench values for unmapped S1/S2/A/D and spare channels, not measured inputs.
  std::array<uint16_t, 18> fixed{{1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000,
    1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 65535, 65535}};
  double deadzone{0.02};

  void validate() const
  {
    if (!std::isfinite(deadzone) || deadzone < 0 || deadzone >= 0.5) {
      throw std::invalid_argument("stick_deadzone must be in [0, 0.5)");
    }
    for (size_t i = 0; i < 8; ++i) {
      if (indices[i] < 0 || indices[i] > 63 ||
        !std::isfinite(signs[i]) || std::abs(signs[i]) != 1 ||
        !std::isfinite(centers[i]) || std::abs(centers[i]) >= 0.5)
      {
        throw std::invalid_argument("Invalid RC axis index, sign or center");
      }
    }
    for (uint16_t value : fixed) {
      if (!((value >= 1000 && value <= 2000) || value == 65535)) {
        throw std::invalid_argument("fixed_channels must be 1000..2000 or 65535");
      }
    }
  }

  bool convert(const std::vector<float> & axes, std::array<uint16_t, 18> & output) const
  {
    output = fixed;
    for (size_t i = 0; i < 8; ++i) {
      if (static_cast<size_t>(indices[i]) >= axes.size()) {return false;}
      double value = axes[indices[i]];
      if (!std::isfinite(value) || std::abs(value) > 1.05) {return false;}
      value = std::clamp(value, -1.0, 1.0);
      if (i < 4) {
        value -= centers[i];
        value /= value >= 0 ? 1 - centers[i] : 1 + centers[i];
        value = std::abs(value) <= deadzone ? 0 :
          std::copysign((std::abs(value) - deadzone) / (1 - deadzone), value);
      } else {
        value = value > 0.5 ? 1 : (value < -0.5 ? -1 : 0);
      }
      output[i] = static_cast<uint16_t>(std::lround(1500 + 500 * signs[i] * value));
    }
    return true;
  }
};
}  // namespace flapping_uav_gcs
#endif  // FLAPPING_UAV_GCS__RC_MAPPING_HPP_
