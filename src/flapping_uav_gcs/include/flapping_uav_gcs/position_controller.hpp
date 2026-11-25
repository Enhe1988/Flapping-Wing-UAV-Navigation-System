#ifndef FLAPPING_UAV_GCS__POSITION_CONTROLLER_HPP_
#define FLAPPING_UAV_GCS__POSITION_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace flapping_uav_gcs
{
using XY = std::array<double, 2>;
struct PositionControlConfig
{
  // Zero means unconfigured. Real gains/limits must come from commissioning.
  double kp{0}, kd{0}, max_speed{0}, max_acceleration{0};
  double position_tolerance{0}, speed_tolerance{0}, dwell_s{0};
  bool continuous_hold{false};
  bool velocity_feedback_required{true};
  // Optional map-axis overrides; old scalar configurations remain valid.
  double kp_x{std::numeric_limits<double>::quiet_NaN()}, kp_y{std::numeric_limits<double>::quiet_NaN()};
  double kd_x{std::numeric_limits<double>::quiet_NaN()}, kd_y{std::numeric_limits<double>::quiet_NaN()};
  double proportional(size_t axis) const
  {const double gain=axis==0?kp_x:kp_y;return std::isnan(gain)?kp:gain;}
  double damping(size_t axis) const
  {const double gain=axis==0?kd_x:kd_y;return std::isnan(gain)?kd:gain;}
  bool configured() const
  {
    return (velocity_feedback_required || (damping(0)==0 && damping(1)==0)) &&
           std::isfinite(proportional(0)) && proportional(0)>0 &&
           std::isfinite(proportional(1)) && proportional(1)>0 &&
           std::isfinite(damping(0)) && damping(0)>=0 &&
           std::isfinite(damping(1)) && damping(1)>=0 &&
           std::isfinite(max_speed) && max_speed > 0 &&
           std::isfinite(max_acceleration) && max_acceleration > 0 &&
           std::isfinite(position_tolerance) && position_tolerance > 0 &&
           std::isfinite(speed_tolerance) && speed_tolerance > 0 &&
           std::isfinite(dwell_s) && dwell_s > 0;
  }
};
struct PositionControlResult
{
  XY velocity{{0, 0}};
  double distance{std::numeric_limits<double>::quiet_NaN()};
  double speed{std::numeric_limits<double>::quiet_NaN()};
  bool valid{false}, reached{false};
};
class PositionController
{
public:
  explicit PositionController(PositionControlConfig config) : config_(config) {}
  bool set_goal(XY goal)
  {
    if (!std::isfinite(goal[0]) || !std::isfinite(goal[1])) {return false;}
    goal_ = goal; has_goal_ = true; reached_ = false; dwell_ = 0;
    return true;
  }
  // Pause an interrupted stream without retaining its last nonzero command.
  void pause() {velocity_ = {{0, 0}}; reached_ = false; dwell_ = 0;}
  void stop() {pause(); has_goal_ = false;}
  bool has_goal() const {return has_goal_;}
  XY goal() const {return goal_;}
  PositionControlResult step(XY position, XY measured_velocity, double dt,
    XY reference_velocity={{0,0}}, bool moving_reference=false)
  {
    PositionControlResult out;
    if (!config_.configured() || !has_goal_ || !std::isfinite(dt) || dt <= 0 ||
      !std::isfinite(position[0]) || !std::isfinite(position[1]) ||
      (config_.velocity_feedback_required &&
      (!std::isfinite(measured_velocity[0]) || !std::isfinite(measured_velocity[1]))) ||
      !std::isfinite(reference_velocity[0]) || !std::isfinite(reference_velocity[1]))
    {stop(); return out;}
    out.distance = std::hypot(goal_[0] - position[0], goal_[1] - position[1]);
    out.speed = config_.velocity_feedback_required ?
      std::hypot(measured_velocity[0], measured_velocity[1]) : std::numeric_limits<double>::quiet_NaN();
    out.valid = true;
    // Limit elapsed time after a stalled executor; never grant a large velocity step.
    dt = std::min(dt, 0.1);
    if (!moving_reference && out.distance <= config_.position_tolerance &&
      (!config_.velocity_feedback_required || out.speed <= config_.speed_tolerance)) {
      dwell_ += dt;
    } else {dwell_ = 0;}
    reached_ = !moving_reference && (config_.continuous_hold ? dwell_ >= config_.dwell_s :
      reached_ || dwell_ >= config_.dwell_s);
    if (reached_) {velocity_ = {{0, 0}}; out.reached = true; return out;}
    const XY feedforward=moving_reference?reference_velocity:XY{{0,0}};
    XY target;
    for(size_t i=0;i<2;++i){
      target[i]=feedforward[i]+config_.proportional(i)*(goal_[i]-position[i]);
      if(config_.velocity_feedback_required){
        target[i]+=config_.damping(i)*(feedforward[i]-measured_velocity[i]);}
    }
    const double speed = std::hypot(target[0], target[1]);
    if (!std::isfinite(speed)) {stop(); out.valid = false; return out;}
    if (speed > config_.max_speed) {
      for (double & v : target) {v *= config_.max_speed / speed;}
    }
    XY delta{{target[0] - velocity_[0], target[1] - velocity_[1]}};
    const double change = std::hypot(delta[0], delta[1]);
    const double allowed = config_.max_acceleration * dt;
    if (change > allowed) {
      for (double & v : delta) {v *= allowed / change;}
    }
    for (size_t i = 0; i < 2; ++i) {velocity_[i] += delta[i];}
    out.velocity = velocity_;
    return out;
  }
private:
  PositionControlConfig config_;
  XY goal_{{0, 0}}, velocity_{{0, 0}};
  bool has_goal_{false}, reached_{false};
  double dwell_{0};
};
}  // namespace flapping_uav_gcs
#endif
