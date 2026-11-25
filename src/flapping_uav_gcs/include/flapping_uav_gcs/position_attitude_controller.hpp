#ifndef FLAPPING_UAV_GCS__POSITION_ATTITUDE_CONTROLLER_HPP_
#define FLAPPING_UAV_GCS__POSITION_ATTITUDE_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include "flapping_uav_gcs/position_controller.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"

namespace flapping_uav_gcs
{
struct PositionAttitudeConfig
{
  double kp_x_deg_per_m{5},kp_y_deg_per_m{5};
  double ki_x_deg_per_m_s{0},ki_y_deg_per_m_s{0};
  double kd_x_deg_s_per_m{0},kd_y_deg_s_per_m{0};
  double integral_limit_deg{2},integral_zone_m{.75},integral_max_speed_mps{.4};
  double derivative_limit_deg{8},velocity_filter_tau_s{.2},velocity_limit_mps{3};
  double sample_max_gap_s{.25},settled_speed_mps{.1};
  double max_angle_deg{30},max_rate_deg_s{60};
  double position_tolerance_m{.05},dwell_s{.5};
  bool configured() const
  {
    const auto positive=[](double v){return std::isfinite(v) && v>0;};
    const auto nonnegative=[](double v){return std::isfinite(v) && v>=0;};
    return positive(kp_x_deg_per_m) && positive(kp_y_deg_per_m) &&
      nonnegative(ki_x_deg_per_m_s) && nonnegative(ki_y_deg_per_m_s) &&
      nonnegative(kd_x_deg_s_per_m) && nonnegative(kd_y_deg_s_per_m) &&
      nonnegative(integral_limit_deg) && positive(integral_zone_m) && positive(integral_max_speed_mps) &&
      positive(derivative_limit_deg) && nonnegative(velocity_filter_tau_s) && positive(velocity_limit_mps) &&
      positive(sample_max_gap_s) && positive(settled_speed_mps) &&
      positive(max_angle_deg) && max_angle_deg<=30 && positive(max_rate_deg_s) &&
      positive(position_tolerance_m) && positive(dwell_s);
  }
};
struct PositionAttitudeResult
{
  double pitch_deg{0},roll_deg{0},distance_m{NAN};
  XY p_map_deg{{0,0}},i_map_deg{{0,0}},d_map_deg{{0,0}},velocity_map_mps{{NAN,NAN}};
  bool valid{false},reached{false},velocity_valid{false},integrating{false},limited{false};
};
class PositionAttitudeController
{
public:
  explicit PositionAttitudeController(PositionAttitudeConfig config):config_(config){}
  bool configured() const {return config_.configured();}
  void reset()
  {
    pitch_=roll_=dwell_=sample_age_=synthetic_time_=0;goal_seen_=sample_seen_=velocity_valid_=false;
    integral_={{0,0}};velocity_={{0,0}};
  }
  // source_time_s is the POSITION stamp, not the 50 Hz controller timer.
  // Omit it only for callers that supply one new measurement on every step.
  PositionAttitudeResult step(XY position,XY goal,double map_to_enu_yaw,
    QuaternionWxyz q,bool body_to_ned,double dt,bool moving_reference=false,double source_time_s=NAN)
  {
    PositionAttitudeResult result;
    if(!configured() || !std::isfinite(dt) || dt<=0 ||
      !std::isfinite(position[0]) || !std::isfinite(position[1]) ||
      !std::isfinite(goal[0]) || !std::isfinite(goal[1])){reset();return result;}
    dt=std::min(dt,.1);
    if(!std::isfinite(source_time_s)){
      if(!std::isnan(source_time_s)){reset();return result;}
      synthetic_time_+=dt;source_time_s=synthetic_time_;
    }
    if(!goal_seen_ || goal!=last_goal_){
      dwell_=0;
      // A new waypoint must not inherit the previous waypoint's bias. Moving
      // references change each tick, so they intentionally retain the bias.
      if(!moving_reference){integral_={{0,0}};}
      last_goal_=goal;goal_seen_=true;
    }
    if(sample_seen_ && source_time_s<sample_time_){reset();return result;}
    sample_age_+=dt;
    const bool new_sample=!sample_seen_ || source_time_s>sample_time_;
    double sample_dt=0;
    if(new_sample){
      if(sample_seen_){sample_dt=source_time_s-sample_time_;}
      if(sample_dt>0 && sample_dt<=config_.sample_max_gap_s){
        XY raw{{(position[0]-sample_position_[0])/sample_dt,(position[1]-sample_position_[1])/sample_dt}};
        if(!std::isfinite(raw[0]) || !std::isfinite(raw[1])){reset();return result;}
        bound_norm(raw,config_.velocity_limit_mps);
        const double alpha=config_.velocity_filter_tau_s==0?1:-std::expm1(-sample_dt/config_.velocity_filter_tau_s);
        for(size_t i=0;i<2;++i){velocity_[i]+=alpha*(raw[i]-velocity_[i]);}
        velocity_valid_=true;
      }else{velocity_={{0,0}};velocity_valid_=false;sample_dt=0;}
      sample_time_=source_time_s;sample_position_=position;sample_seen_=true;sample_age_=0;
    }
    // A held pose is never differentiated as a new zero-speed measurement.
    // Stop using the old derivative during a gap, and re-prime on recovery.
    if(sample_age_>config_.sample_max_gap_s){velocity_valid_=false;velocity_={{0,0}};}
    result.velocity_valid=velocity_valid_;
    if(velocity_valid_){result.velocity_map_mps=velocity_;}
    const XY error{{goal[0]-position[0],goal[1]-position[1]}};
    result.distance_m=std::hypot(error[0],error[1]);
    result.p_map_deg={{config_.kp_x_deg_per_m*error[0],config_.kp_y_deg_per_m*error[1]}};
    if(velocity_valid_){result.d_map_deg={{-config_.kd_x_deg_s_per_m*velocity_[0],-config_.kd_y_deg_s_per_m*velocity_[1]}};}
    bound_norm(result.d_map_deg,config_.derivative_limit_deg);
    const auto rotate=[&](XY demand,XY & angles){
      Velocity3 body;
      if(!map_velocity_to_heading({{demand[0],demand[1],0}},map_to_enu_yaw,q,body_to_ned,body)){return false;}
      angles={{body[0],-body[1]}};return true;
    };
    const XY pd{{result.p_map_deg[0]+result.d_map_deg[0],result.p_map_deg[1]+result.d_map_deg[1]}};
    XY requested;
    if(!rotate({{pd[0]+integral_[0],pd[1]+integral_[1]}},requested)){reset();return result;}
    const double change=config_.max_rate_deg_s*dt;
    const XY lower{{std::max(-config_.max_angle_deg,pitch_-change),std::max(-config_.max_angle_deg,roll_-change)}};
    const XY upper{{std::min(config_.max_angle_deg,pitch_+change),std::min(config_.max_angle_deg,roll_+change)}};
    if(new_sample && sample_dt>0 && velocity_valid_ &&
      result.distance_m<=config_.integral_zone_m &&
      std::hypot(velocity_[0],velocity_[1])<=config_.integral_max_speed_mps){
      // Conditional integration in map axes, checked AFTER rotation against
      // both FC angle saturation and slew limits; allow unwinding at a limit.
      const XY ki{{config_.ki_x_deg_per_m_s,config_.ki_y_deg_per_m_s}};
      for(size_t i=0;i<2;++i){
        XY candidate=integral_;candidate[i]+=ki[i]*error[i]*sample_dt;
        bound_norm(candidate,config_.integral_limit_deg);
        XY next;
        if(!rotate({{pd[0]+candidate[0],pd[1]+candidate[1]}},next)){reset();return result;}
        bool accept=true;
        for(size_t j=0;j<2;++j){
          const double excess=next[j]-std::clamp(next[j],lower[j],upper[j]);
          if((next[j]-requested[j])*excess>1e-12){accept=false;}
        }
        if(accept){
          result.integrating=result.integrating || candidate!=integral_;
          integral_=candidate;requested=next;
        }
      }
    }
    if(!std::isfinite(requested[0]) || !std::isfinite(requested[1])){reset();return result;}
    result.i_map_deg=integral_;
    const bool damping_enabled=config_.kd_x_deg_s_per_m>0 || config_.kd_y_deg_s_per_m>0;
    const bool slow_enough=!damping_enabled || (velocity_valid_ &&
      std::hypot(velocity_[0],velocity_[1])<=config_.settled_speed_mps);
    if(!moving_reference && result.distance_m<=config_.position_tolerance_m && slow_enough){dwell_+=dt;}
    else{dwell_=0;}
    result.valid=true;result.reached=!moving_reference && dwell_>=config_.dwell_s;
    // Reached is a status, not a command reset: keep I for trim and D for
    // braking even inside tolerance. No fabricated optical-flow feedback.
    pitch_=std::clamp(requested[0],lower[0],upper[0]);
    roll_=std::clamp(requested[1],lower[1],upper[1]);
    result.limited=std::abs(pitch_-requested[0])>1e-9 || std::abs(roll_-requested[1])>1e-9;
    result.pitch_deg=pitch_;result.roll_deg=roll_;
    return result;
  }
private:
  static void bound_norm(XY & value,double limit)
  {
    const double norm=std::hypot(value[0],value[1]);
    if(norm>limit){for(auto & component:value){component*=limit/norm;}}
  }
  PositionAttitudeConfig config_;
  double pitch_{0},roll_{0},dwell_{0},sample_time_{0},sample_age_{0},synthetic_time_{0};
  bool goal_seen_{false},sample_seen_{false},velocity_valid_{false};
  XY last_goal_{{0,0}},sample_position_{{0,0}},velocity_{{0,0}},integral_{{0,0}};
};
}  // namespace flapping_uav_gcs
#endif
