#ifndef FLAPPING_UAV_GCS__SCENE_GEOMETRY_HPP_
#define FLAPPING_UAV_GCS__SCENE_GEOMETRY_HPP_
#include "flapping_uav_gcs/velocity_rotation.hpp"
namespace flapping_uav_gcs
{
inline QuaternionWxyz multiply_quaternions(const QuaternionWxyz & a, const QuaternionWxyz & b)
{
  return {{a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
    a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
    a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
    a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]}};
}
// Full roll/pitch/yaw for display. Unlike the planar controller, this must not
// discard tilt. q_map_body = q_map_ENU * q_ENU_NED * q_NED_body.
inline bool body_attitude_to_map(
  QuaternionWxyz q, bool q_body_to_ned, double map_to_enu_yaw, QuaternionWxyz & result)
{
  if (!std::isfinite(map_to_enu_yaw) || !normalize_attitude(q)) {return false;}
  if (!q_body_to_ned) {for (size_t i = 1; i < 4; ++i) {q[i] = -q[i];}}
  const double h = std::sqrt(.5);
  const QuaternionWxyz map_enu{{std::cos(map_to_enu_yaw/2), 0, 0, -std::sin(map_to_enu_yaw/2)}};
  result = multiply_quaternions(map_enu, multiply_quaternions({{0, h, h, 0}}, q));
  return normalize_attitude(result);
}
inline Velocity3 uwb_point_to_map(const Velocity3 & p, double yaw, const Velocity3 & translation)
{
  const double c=std::cos(yaw), s=std::sin(yaw);
  return {{c*p[0]-s*p[1]+translation[0], s*p[0]+c*p[1]+translation[1], p[2]+translation[2]}};
}
}  // namespace flapping_uav_gcs
#endif
