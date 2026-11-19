#ifndef FLAPPING_UAV_GCS__VELOCITY_ROTATION_HPP_
#define FLAPPING_UAV_GCS__VELOCITY_ROTATION_HPP_

#include <array>
#include <cmath>

namespace flapping_uav_gcs
{
using Velocity3 = std::array<double, 3>;
using QuaternionWxyz = std::array<double, 4>;

inline bool finite_vector(const Velocity3 & value)
{
  for (double item : value) {
    if (!std::isfinite(item)) {
      return false;
    }
  }
  return true;
}

inline bool normalize_attitude(QuaternionWxyz & q)
{
  double squared = 0;
  for (double item : q) {
    if (!std::isfinite(item)) {return false;}
    squared += item * item;
  }
  const double norm = std::sqrt(squared);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.05) {return false;}
  for (double & item : q) {
    item /= norm;
  }
  return true;
}

// Rotate a vector actively by a unit WXYZ quaternion.
inline Velocity3 rotate_vector(const QuaternionWxyz & q, const Velocity3 & v)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {{
    (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] +
    2 * (x * z + w * y) * v[2],
    2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] +
    2 * (y * z - w * x) * v[2],
    2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] +
    (1 - 2 * (x * x + y * y)) * v[2]}};
}

// map -> ENU: fixed yaw about +Z, followed by ENU -> NED [N,E,D]=[y,x,-z].
// Translation is intentionally absent: this transforms a velocity vector.
inline bool map_velocity_to_body(
  const Velocity3 & map_velocity, double map_to_enu_yaw,
  QuaternionWxyz q, bool quaternion_body_to_ned, Velocity3 & output)
{
  output = {{0, 0, 0}};
  if (!finite_vector(map_velocity) || !std::isfinite(map_to_enu_yaw) ||
    !normalize_attitude(q)) {return false;}
  const double c = std::cos(map_to_enu_yaw), s = std::sin(map_to_enu_yaw);
  const Velocity3 ned{{s * map_velocity[0] + c * map_velocity[1],
    c * map_velocity[0] - s * map_velocity[1], -map_velocity[2]}};
  // Need NED->body. Invert a body->NED quaternion; use NED->body as received.
  if (quaternion_body_to_ned) {
    for (size_t i = 1; i < 4; ++i) {
      q[i] = -q[i];
    }
  }
  output = rotate_vector(q, ned);
  return finite_vector(output);
}

// Invert the map/ENU/NED alignment for a complete NED velocity.
inline Velocity3 ned_velocity_to_map(const Velocity3 & ned, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {{c * ned[1] + s * ned[0], -s * ned[1] + c * ned[0], -ned[2]}};
}

// Horizontal forward/right setpoints for the user-confirmed planar FC velocity
// loop. This is a heading frame, not the complete tilted BODY_FRD frame.
inline bool map_velocity_to_heading(
  const Velocity3 & map_velocity, double map_to_enu_yaw, QuaternionWxyz q,
  bool q_body_to_ned, Velocity3 & output)
{
  output = {{0, 0, 0}};
  if (!finite_vector(map_velocity) || map_velocity[2] != 0 || !normalize_attitude(q)) {
    return false;
  }
  if (!q_body_to_ned) {for (size_t i = 1; i < 4; ++i) {q[i] = -q[i];}}
  const auto forward = rotate_vector(q, {{1, 0, 0}});
  if (std::hypot(forward[0], forward[1]) < 1e-3) {return false;}
  const double half_yaw = std::atan2(forward[1], forward[0]) / 2;
  return map_velocity_to_body(map_velocity, map_to_enu_yaw,
    {{std::cos(half_yaw), 0, 0, std::sin(half_yaw)}}, true, output);
}

// ID250 has only TWO velocity components; its z field must never enter this function.
// heading=true: tilt-compensated horizontal forward/right feedback.
// heading=false: true body FRD components, recover missing body vz ONLY under an
// explicit zero-world-vertical-speed assumption. Near-horizontal body Z is singular.
inline bool feedback_xy_to_map(
  double forward, double right, double yaw, QuaternionWxyz q,
  bool q_body_to_ned, bool heading, bool assume_zero_world_vz, Velocity3 & output)
{
  output = {{0, 0, 0}};
  if (!std::isfinite(forward) || !std::isfinite(right) || !std::isfinite(yaw) ||
    !normalize_attitude(q)) {return false;}
  if (!q_body_to_ned) {for (size_t i = 1; i < 4; ++i) {q[i] = -q[i];}}
  const auto ex = rotate_vector(q, {{1, 0, 0}});
  const auto ey = rotate_vector(q, {{0, 1, 0}});
  const auto ez = rotate_vector(q, {{0, 0, 1}});
  Velocity3 ned;
  if (heading) {
    const double norm = std::hypot(ex[0], ex[1]);
    if (norm < 1e-3) {return false;}
    const double c = ex[0] / norm, s = ex[1] / norm;
    ned = {{c * forward - s * right, s * forward + c * right, 0}};
  } else {
    if (!assume_zero_world_vz || std::abs(ez[2]) < 1e-3) {return false;}
    const double vz = -(ex[2] * forward + ey[2] * right) / ez[2];
    ned = rotate_vector(q, {{forward, right, vz}});
  }
  output = ned_velocity_to_map(ned, yaw);
  return finite_vector(output);
}
}  // namespace flapping_uav_gcs
#endif  // FLAPPING_UAV_GCS__VELOCITY_ROTATION_HPP_
