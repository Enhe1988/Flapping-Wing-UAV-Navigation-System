#ifndef FLAPPING_UAV_GCS__LOOSE_ODOMETRY_HPP_
#define FLAPPING_UAV_GCS__LOOSE_ODOMETRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include "flapping_uav_gcs/uwb_lowpass.hpp"

namespace flapping_uav_gcs
{
struct LooseOdometryConfig
{
  double uwb_timeout_s{.5}, flow_timeout_s{.3}, max_dead_reckoning_s{2.};
  double jump_slack_m{.3}, max_uwb_speed_mps{3.}, innovation_gate_m{.75};
  double uwb_correction_gain{1.}, initial_variance_m2{.0025}, variance_growth_m2_s{.04};
  int confirmation_samples{3};
  double uwb_sync_period_s{0.}, recovery_confirmation_s{0.};
  int recovery_confirmation_samples{0};  // 0 reuses initialization count.
  bool uwb_filter_enabled{false};
  double uwb_filter_tau_s{.1};
  bool valid() const
  {
    return std::isfinite(uwb_timeout_s) && uwb_timeout_s > 0 &&
      std::isfinite(flow_timeout_s) && flow_timeout_s > 0 &&
      std::isfinite(max_dead_reckoning_s) && max_dead_reckoning_s > uwb_timeout_s &&
      std::isfinite(jump_slack_m) && jump_slack_m > 0 &&
      std::isfinite(max_uwb_speed_mps) && max_uwb_speed_mps > 0 &&
      std::isfinite(innovation_gate_m) && innovation_gate_m > 0 &&
      std::isfinite(uwb_correction_gain) && uwb_correction_gain > 0 && uwb_correction_gain <= 1 &&
      std::isfinite(initial_variance_m2) && initial_variance_m2 > 0 &&
      std::isfinite(variance_growth_m2_s) && variance_growth_m2_s >= 0 && confirmation_samples > 0 &&
      std::isfinite(uwb_sync_period_s) && uwb_sync_period_s>=0 &&
      std::isfinite(recovery_confirmation_s) && recovery_confirmation_s>=0 && recovery_confirmation_samples>=0 &&
      std::isfinite(uwb_filter_tau_s) && uwb_filter_tau_s>0;
  }
};
struct LooseOdometryEstimate
{
  bool initialized{false}, valid{false}, flow_valid{false}, uwb_recent{false}, propagation_complete{false};
  std::string state{"WAITING_UWB"}, reason{"Waiting for confirmed UWB fixes"};
  std::array<double, 2> position{{0,0}}, velocity{{0,0}};
  std::array<double, 3> covariance{{.0025,0,.0025}};  // xx, xy, yy, model-aided
  double position_time{0}, uwb_age{INFINITY}, flow_age{INFINITY}, innovation{NAN};
  uint64_t accepted{0}, rejected{0};
  std::array<double,2> reliable_uwb{{NAN,NAN}};
  uint64_t syncs{0};
};
// Independent XY complementary observer. All times use one host monotonic clock.
// Invalid inputs never fabricate zero-velocity measurements; they stop prediction.
class LooseOdometry
{
public:
  explicit LooseOdometry(LooseOdometryConfig config) : config_(config),
    smoother_(config.uwb_filter_enabled,config.uwb_filter_tau_s,config.uwb_timeout_s)
  {if (!config_.valid()) {throw std::invalid_argument("Invalid loose odometry configuration");}}
  void reset()
  {
    initialized_=false; flow_seen_=false; flow_good_=false; uwb_seen_=false;
    timeline_seen_=false; propagation_complete_=false; uwb_problem_=false; candidate_count_=0;
    last_position_={{0,0}}; covariance_={{config_.initial_variance_m2,0,config_.initial_variance_m2}};
    innovation_=NAN; problem_="Waiting for confirmed UWB fixes";
    smoother_.reset();
  }
  void predict(double now)
  {
    if (!std::isfinite(now)) {return;}
    if (!timeline_seen_) {timeline_=now; timeline_seen_=true; return;}
    if (now < timeline_) {return;}
    if (initialized_ && flow_good_) {
      const double until=std::min({now, flow_time_+config_.flow_timeout_s,
        uwb_time_+config_.max_dead_reckoning_s});
      if (until>timeline_ && propagation_complete_) {
        const double dt=until-timeline_;
        last_position_[0]+=velocity_[0]*dt; last_position_[1]+=velocity_[1]*dt;
        covariance_[0]+=config_.variance_growth_m2_s*dt;
        covariance_[2]+=config_.variance_growth_m2_s*dt;
        position_time_=until;
      }
      if (now>flow_time_+config_.flow_timeout_s+1e-9) {propagation_complete_=false;}
    }
    if (initialized_ && !flow_good_ && now>timeline_+1e-9) {propagation_complete_=false;}
    timeline_=now;
  }
  void set_flow(double now, const std::array<double,2> & velocity, double received_time=NAN)
  {
    predict(now);
    if (!finite(velocity)) {invalidate_flow(now,"Nonfinite optical-flow velocity"); return;}
    velocity_=velocity; flow_time_=std::isfinite(received_time)?std::min(now,received_time):now;
    flow_seen_=true; flow_good_=true; flow_problem_.clear();
  }
  void invalidate_flow(double now, const std::string & why)
  {
    predict(now); flow_good_=false; propagation_complete_=false; flow_problem_=why;
  }
  void unavailable_uwb(double now, const std::string & why)
  {
    predict(now); uwb_problem_=true; candidate_count_=0; problem_=why;
  }
  bool correct(double now, const std::array<double,2> & point,
    const std::array<double,3> * covariance=nullptr)
  {
    predict(now);
    if (!finite(point)) {return reject("Nonfinite UWB XY");}
    const bool first=!initialized_;bool recovering=false;
    if (first) {
      if (!confirm(now,point,false)) {return false;}
    } else {
      const double dt=std::max(0.,now-uwb_time_);
      const double step=std::hypot(point[0]-last_uwb_[0],point[1]-last_uwb_[1]);
      innovation_=std::hypot(point[0]-last_position_[0],point[1]-last_position_[1]);
      if (step>config_.jump_slack_m+config_.max_uwb_speed_mps*dt) {
        return reject("UWB step exceeds displacement/speed gate");
      }
      if (innovation_>config_.innovation_gate_m) {
        return reject("UWB innovation too large; retain estimate, inspect or /fusion/reset");
      }
      recovering=uwb_problem_ || now-uwb_time_>config_.uwb_timeout_s;
      if (recovering && !confirm(now,point,true)) {return false;}
    }
    // Gates compare RAW fixes; rejected/candidate points never enter smoothing.
    if(!smoother_.update(now,point,first||recovering)){return reject("UWB filter input time invalid");}
    const auto & filtered=smoother_.value();
    if(first){last_position_=filtered;initialized_=true;sync(now,covariance);}
    else if(recovering || !propagation_complete_ || now-last_sync_>=config_.uwb_sync_period_s){
      for(size_t i=0;i<2;++i){
        last_position_[i]+=config_.uwb_correction_gain*(filtered[i]-last_position_[i]);
      }
      sync(now,covariance);
    }
    last_uwb_=point; uwb_time_=now; uwb_seen_=true; uwb_problem_=false;
    propagation_complete_=true; candidate_count_=0; ++accepted_;
    problem_.clear();
    return true;
  }
  LooseOdometryEstimate estimate(double now)
  {
    predict(now); LooseOdometryEstimate out;
    out.initialized=initialized_; out.position=last_position_; out.covariance=covariance_;
    out.position_time=position_time_;out.accepted=accepted_;out.rejected=rejected_;out.innovation=innovation_;
    out.reliable_uwb=uwb_seen_?smoother_.value():std::array<double,2>{{NAN,NAN}};out.syncs=syncs_;
    out.uwb_age=uwb_seen_?std::max(0.,now-uwb_time_):INFINITY;
    out.flow_age=flow_seen_?std::max(0.,now-flow_time_):INFINITY;
    out.flow_valid=flow_good_ && out.flow_age<=config_.flow_timeout_s;
    out.uwb_recent=uwb_seen_ && out.uwb_age<=config_.uwb_timeout_s && !uwb_problem_;
    out.propagation_complete=propagation_complete_;
    if (!initialized_) {
      out.state="WAITING_UWB";out.reason=problem_;return out;
    }
    if (out.uwb_recent) {
      out.valid=true;out.state=out.flow_valid && propagation_complete_?"FUSED":"UWB_ONLY";
      out.reason=out.flow_valid && propagation_complete_?"Flow prediction with accepted UWB correction":"UWB XY available; flow prediction unavailable";
    } else if (out.flow_valid && propagation_complete_ && out.uwb_age<=config_.max_dead_reckoning_s) {
      out.valid=true;out.state="DEAD_RECKONING";
      out.reason=problem_.empty()?"UWB stale; short optical-flow prediction":"UWB rejected: "+problem_;
    } else {
      out.state="INVALID";
      if (out.uwb_age>config_.max_dead_reckoning_s) {out.reason="UWB outage exceeds dead-reckoning limit";}
      else if (!out.flow_valid) {out.reason="UWB unavailable; flow missing/stale/invalid: "+flow_problem_;}
      else {out.reason="Unobserved flow interval; fresh UWB re-anchor required";}
    }
    if (out.valid && out.flow_valid) {out.velocity=velocity_;}
    return out;
  }
private:
  static bool finite(const std::array<double,2> & p) {return std::isfinite(p[0]) && std::isfinite(p[1]);}
  static bool covariance_valid(const std::array<double,3> & p)
  {return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
    p[0]>=0 && p[2]>=0 && p[0]*p[2]>=p[1]*p[1];}
  bool reject(const std::string & why)
  {++rejected_;candidate_count_=0;uwb_problem_=true;problem_=why;return false;}
  void sync(double now,const std::array<double,3> * covariance)
  {
    last_sync_=position_time_=now;++syncs_;
    covariance_={{config_.initial_variance_m2,0,config_.initial_variance_m2}};
    if(covariance && covariance_valid(*covariance)){covariance_=*covariance;}
  }
  bool confirm(double now,const std::array<double,2> & point,bool recovering)
  {
    if (candidate_count_>0 && (now-candidate_time_>config_.uwb_timeout_s ||
      std::hypot(point[0]-candidate_[0],point[1]-candidate_[1])>
      config_.jump_slack_m+config_.max_uwb_speed_mps*std::max(0.,now-candidate_time_))) {candidate_count_=0;}
    if(candidate_count_==0){candidate_started_=now;}
    candidate_=point;candidate_time_=now;++candidate_count_;
    const int count=recovering && config_.recovery_confirmation_samples>0?config_.recovery_confirmation_samples:config_.confirmation_samples;
    if (candidate_count_<count || (recovering && now-candidate_started_<config_.recovery_confirmation_s)) {
      problem_="Confirming UWB fixes "+std::to_string(candidate_count_)+"/"+std::to_string(count);
      return false;
    }
    return true;
  }
  LooseOdometryConfig config_;
  UwbLowPass smoother_;
  bool initialized_{false},flow_seen_{false},flow_good_{false},uwb_seen_{false},timeline_seen_{false};
  bool propagation_complete_{false},uwb_problem_{false};
  double timeline_{0},position_time_{0},flow_time_{0},uwb_time_{0},candidate_time_{0},innovation_{NAN};
  std::array<double,2> last_position_{{0,0}},last_uwb_{{0,0}},velocity_{{0,0}},candidate_{{0,0}};
  std::array<double,3> covariance_{{.0025,0,.0025}};
  int candidate_count_{0};uint64_t accepted_{0},rejected_{0};
  double last_sync_{0},candidate_started_{0};uint64_t syncs_{0};
  std::string problem_{"Waiting for confirmed UWB fixes"},flow_problem_{"No valid flow sample"};
};
}  // namespace flapping_uav_gcs
#endif
