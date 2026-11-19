#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/loose_odometry.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/fusion_status.hpp"
#include "flapping_uav_msgs/msg/heading_alignment.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace flapping_uav_gcs
{
class FusionOdometryNode : public rclcpp::Node
{
public:
  explicit FusionOdometryNode(const rclcpp::NodeOptions & options) : Node("fusion_odometry_node",options)
  {
    frame_=parameter("map_frame_id",std::string("map"));
    child_=parameter("child_frame_id",std::string("fusion_map_axes"));
    config_.uwb_timeout_s=parameter("uwb_timeout_ms",500)*.001;
    config_.flow_timeout_s=parameter("flow_timeout_ms",300)*.001;
    config_.max_dead_reckoning_s=parameter("max_dead_reckoning_s",2.);
    config_.jump_slack_m=parameter("uwb_jump_slack_m",.3);
    config_.max_uwb_speed_mps=parameter("max_uwb_speed_mps",3.);
    config_.innovation_gate_m=parameter("uwb_innovation_gate_m",.75);
    config_.uwb_correction_gain=parameter("uwb_correction_gain",1.);
    config_.confirmation_samples=parameter("uwb_confirmation_samples",3);
    config_.initial_variance_m2=parameter("initial_position_variance_m2",.0025);
    config_.variance_growth_m2_s=parameter("position_variance_growth_m2_per_s",.04);
    attitude_timeout_=parameter("attitude_timeout_ms",300);
    pair_skew_limit_=parameter("max_attitude_pair_skew_ms",250);
    device_gap_limit_=parameter("max_flow_device_gap_ms",300);
    max_speed_=parameter("max_flow_speed_mps",5.);
    system_=parameter("target_system",1);component_=parameter("target_component",1);
    use_navigation_alignment_=parameter("use_navigation_alignment",true);
    q_body_to_ned_=parameter("quaternion_body_to_ned",true);
    path_limit_=parameter("max_path_points",3000);
    path_rate_=parameter("path_publish_rate_hz",5.);
    const double rate=parameter("publish_rate_hz",50.);
    if (frame_.empty() || child_.empty() || frame_==child_ || !config_.valid() ||
      attitude_timeout_<=0 || pair_skew_limit_<=0 || device_gap_limit_<=0 ||
      !std::isfinite(max_speed_) || max_speed_<=0 || path_limit_<1 || !std::isfinite(path_rate_) || path_rate_<=0 || !std::isfinite(rate) || rate<1 || rate>200 ||
      system_<1 || system_>255 || component_<0 || component_>255) {
      throw std::invalid_argument("Invalid independent fusion odometry configuration");
    }
    alignment_session_.load(*this);
    filter_=std::make_unique<LooseOdometry>(config_);
    odometry_=create_publisher<nav_msgs::msg::Odometry>("/fusion/odometry",10);
    position_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fusion/position",10);
    status_=create_publisher<flapping_uav_msgs::msg::FusionStatus>("/fusion/status",10);
    path_output_=create_publisher<nav_msgs::msg::Path>("/fusion/path",rclcpp::QoS(1).transient_local());
    markers_=create_publisher<visualization_msgs::msg::MarkerArray>("/fusion/markers",rclcpp::QoS(1).transient_local());
    pose_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position",rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
        const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        if (pose_seen_ && stamp<=pose_stamp_) {++duplicate_uwb_;return;}
        pose_=*m;pose_stamp_=stamp;pose_received_=time();pose_seen_=true;process_pose(time());
      });
    uwb_sub_=create_subscription<flapping_uav_msgs::msg::UwbStatus>("/uwb/status",10,
      [this](flapping_uav_msgs::msg::UwbStatus::ConstSharedPtr m){
        uwb_=*m;uwb_received_=time();uwb_seen_=true;process_pose(time());
      });
    attitude_sub_=create_subscription<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude",rclcpp::QoS(10),
      [this](flapping_uav_msgs::msg::AttitudeState::ConstSharedPtr m){
        if (m->system_id!=system_ || m->component_id!=component_) {return;}
        const double t=time();
        if (attitude_seen_ && (m->sample_time_us<attitude_.sample_time_us || m->reset_counter!=attitude_.reset_counter)) {
          alignment_valid_=false;alignment_reason_="FC attitude clock/reference reset; realign fusion";
          attitudes_.clear();filter_->invalidate_flow(t,alignment_reason_);
        }
        if (!attitude_seen_ || m->sample_time_us!=attitude_.sample_time_us || m->reset_counter!=attitude_.reset_counter) {
          attitude_received_=t;attitude_seen_=true;attitudes_.push_back({*m,t});
          while(attitudes_.size()>200 || (!attitudes_.empty() && t-attitudes_.front().received>2.)) {attitudes_.pop_front();}
        }
        attitude_=*m;process_flow(t);
      });
    flow_sub_=create_subscription<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion",rclcpp::QoS(1),
      [this](flapping_uav_msgs::msg::BodyMotionState::ConstSharedPtr m){
        if (m->system_id!=system_ || m->component_id!=component_) {return;}
        const double t=time();
        if (motion_seen_ && m->sample_time_us==motion_.sample_time_us) {++duplicate_flow_;return;}
        if (motion_seen_ && m->sample_time_us<motion_.sample_time_us) {
          alignment_valid_=false;alignment_reason_="FC flow clock reset; realign fusion";
          filter_->invalidate_flow(t,alignment_reason_);++flow_gaps_;
        } else if (motion_seen_ && m->sample_time_us-motion_.sample_time_us>static_cast<uint64_t>(device_gap_limit_)*1000) {
          filter_->invalidate_flow(t,"Flow device-time gap; UWB re-anchor required");++flow_gaps_;
        }
        motion_=*m;motion_received_=t;motion_seen_=true;motion_pending_=true;process_flow(t);
      });
    alignment_sub_=create_subscription<flapping_uav_msgs::msg::HeadingAlignment>("/navigation/heading_alignment",
      rclcpp::QoS(1).reliable().transient_local(),[this](flapping_uav_msgs::msg::HeadingAlignment::ConstSharedPtr m){
        if (!use_navigation_alignment_ || local_alignment_) {return;}
        if (!m->valid || m->header.frame_id!=frame_ || !std::isfinite(m->map_to_enu_yaw_rad)) {
          alignment_valid_=false;alignment_reason_="Navigation alignment invalid/unavailable";
          filter_->invalidate_flow(time(),alignment_reason_);return;
        }
        AlignmentSessionConfig ref;ref.required=true;ref.device_time_us=m->device_time_us;
        ref.host_monotonic_ns=m->host_monotonic_ns;ref.host_boot_id=m->host_boot_id;
        ref.max_clock_skew_ms=m->max_clock_skew_ms;alignment_session_.set_reference(ref);
        yaw_=m->map_to_enu_yaw_rad;q_body_to_ned_=m->quaternion_body_to_ned;alignment_valid_=true;
        alignment_reason_="Read-only navigation heading imported";
        filter_->invalidate_flow(time(),"Heading reference changed; next UWB fix re-anchors prediction");
        motion_pending_=false;
      });
    align_service_=create_service<std_srvs::srv::Trigger>("/fusion/align_positive_x",[this](
      std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        const double t=time();
        if (!attitude_basic(t)) {response->success=false;response->message="Fresh valid FC attitude required";return;}
        const auto & o=attitude_.orientation;QuaternionWxyz q{{o.w,o.x,o.y,o.z}};
        if (!normalize_attitude(q)) {response->success=false;response->message="Invalid quaternion";return;}
        if (!q_body_to_ned_) {for(size_t i=1;i<4;++i){q[i]=-q[i];}}
        const auto forward=rotate_vector(q,{{1,0,0}});
        if (std::hypot(forward[0],forward[1])<1e-3 || !alignment_session_.bind(attitude_.sample_time_us)) {
          response->success=false;response->message="Cannot bind fusion heading/session";return;
        }
        const double angle=std::acos(-1.)/2-std::atan2(forward[1],forward[0]);
        yaw_=std::atan2(std::sin(angle),std::cos(angle));alignment_valid_=true;local_alignment_=true;
        alignment_reason_="Independent fusion +X heading recorded";
        filter_->invalidate_flow(t,"Fusion heading changed; next UWB fix re-anchors prediction");motion_pending_=false;
        response->success=true;response->message=alignment_reason_+"; offset="+std::to_string(yaw_)+" rad; navigation untouched";
        RCLCPP_INFO(get_logger(),"%s",response->message.c_str());
      });
    reset_service_=create_service<std_srvs::srv::Trigger>("/fusion/reset",[this](
      std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        filter_->reset();pose_seen_=false;pose_stamp_=processed_pose_stamp_=0;motion_seen_=motion_pending_=false;
        path_.poses.clear();path_.header.frame_id=frame_;path_.header.stamp=now();path_output_->publish(path_);
        last_published_time_=-1.;last_path_publish_=-INFINITY;response->success=true;
        response->message="Fusion XY reset; waiting for confirmed UWB fixes; navigation untouched";
      });
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1/rate)),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Independent XY fusion observer; no navigation/FC output; DR horizon %.2fs",config_.max_dead_reckoning_s);
  }
private:
  struct TimedAttitude {flapping_uav_msgs::msg::AttitudeState value;double received;};
  static double time() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  template<typename T>T parameter(const std::string & name,const T & value)
  {rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;return declare_parameter<T>(name,value,d);}
  bool fresh_header(const std_msgs::msg::Header & h,double timeout_ms) const
  {
    const auto stamp=rclcpp::Time(h.stamp).nanoseconds();
    const double age=(now().nanoseconds()-stamp)/1e6;
    return stamp>0 && age>=-100 && age<=timeout_ms;
  }
  bool attitude_basic(double t) const
  {
    return attitude_seen_ && (t-attitude_received_)*1000<=attitude_timeout_ && attitude_.valid &&
      attitude_.sample_time_us>0 && attitude_.header.frame_id=="local_ned" && attitude_.body_frame_id=="base_link_frd" &&
      fresh_header(attitude_.header,attitude_timeout_);
  }
  bool heading_ready(double t) const
  {return alignment_valid_ && attitude_basic(t) && alignment_session_.valid(attitude_.sample_time_us);}
  bool uwb_inputs_ready(double t) const
  {
    return uwb_seen_ && pose_seen_ && t-uwb_received_<=config_.uwb_timeout_s &&
      t-pose_received_<=config_.uwb_timeout_s && uwb_.position_valid && uwb_.coordinates_confirmed &&
      uwb_.header.frame_id==frame_ && pose_.header.frame_id==frame_ &&
      fresh_header(uwb_.header,config_.uwb_timeout_s*1000) && fresh_header(pose_.header,config_.uwb_timeout_s*1000);
  }
  void process_pose(double t)
  {
    if (!uwb_inputs_ready(t) || pose_stamp_<=processed_pose_stamp_) {return;}
    processed_pose_stamp_=pose_stamp_;
    const auto & p=pose_.pose.pose.position;
    std::array<double,3> cov{{uwb_.covariance_xy[0],uwb_.covariance_xy[1],uwb_.covariance_xy[3]}};
    filter_->correct(t,{{p.x,p.y}},uwb_.covariance_valid?&cov:nullptr);
  }
  void process_flow(double t)
  {
    if (!heading_ready(t)) {filter_->invalidate_flow(t,alignment_valid_?"Attitude stale/invalid or alignment session expired":alignment_reason_);return;}
    // Pairing a delayed sample must not grant a new freshness window.
    if (motion_seen_ && (t-motion_received_>config_.flow_timeout_s ||
      !fresh_header(motion_.header,config_.flow_timeout_s*1000))) {
      filter_->invalidate_flow(t,"Flow reception stale");motion_pending_=false;return;
    }
    if (!motion_pending_) {return;}
    if (!motion_.velocity_valid || motion_.sample_time_us==0 || motion_.header.frame_id!="body_xy" ||
      !std::isfinite(motion_.forward_velocity_mps) || !std::isfinite(motion_.right_velocity_mps) ||
      std::hypot(motion_.forward_velocity_mps,motion_.right_velocity_mps)>max_speed_) {
      filter_->invalidate_flow(t,"Flow XY invalid, wrong frame, or speed gate exceeded");motion_pending_=false;return;
    }
    const TimedAttitude * nearest=nullptr;double skew=INFINITY;
    for (const auto & a:attitudes_) {
      if (!a.value.valid || a.value.header.frame_id!="local_ned" || a.value.body_frame_id!="base_link_frd" ||
        (t-a.received)*1000>attitude_timeout_ || !fresh_header(a.value.header,attitude_timeout_)) {continue;}
      const double candidate=std::abs(static_cast<long double>(motion_.sample_time_us)-a.value.sample_time_us)/1000.;
      if(candidate<skew){skew=candidate;nearest=&a;}
    }
    pair_skew_=skew;
    if (!nearest || skew>pair_skew_limit_ || !alignment_session_.valid(nearest->value.sample_time_us)) {
      filter_->invalidate_flow(t,"Flow/attitude device timestamps do not match");return;
    }
    const auto & q=nearest->value.orientation;Velocity3 v;
    if (!feedback_xy_to_map(motion_.forward_velocity_mps,motion_.right_velocity_mps,yaw_,
      {{q.w,q.x,q.y,q.z}},q_body_to_ned_,true,false,v)) {
      filter_->invalidate_flow(t,"Cannot rotate flow to map");motion_pending_=false;return;
    }
    filter_->set_flow(t,{{v[0],v[1]}},motion_received_);motion_pending_=false;
  }
  void tick()
  {
    const double t=time();process_pose(t);process_flow(t);
    if (!uwb_inputs_ready(t)) {filter_->unavailable_uwb(t,"UWB input/status stale, invalid, or coordinates unconfirmed");}
    auto estimate=filter_->estimate(t);
    flapping_uav_msgs::msg::FusionStatus status;
    status.header.stamp=now();status.header.frame_id=frame_;status.state=estimate.state;status.reason=estimate.reason;
    status.initialized=estimate.initialized;status.valid=estimate.valid;status.uwb_recent=estimate.uwb_recent;
    status.flow_valid=estimate.flow_valid;status.propagation_complete=estimate.propagation_complete;
    status.alignment_valid=heading_ready(t);status.covariance_is_model=true;
    status.position.x=estimate.position[0];status.position.y=estimate.position[1];
    status.velocity_map.x=estimate.velocity[0];status.velocity_map.y=estimate.velocity[1];
    status.uwb_age_ms=estimate.uwb_age*1000;status.flow_age_ms=estimate.flow_age*1000;
    status.attitude_age_ms=attitude_seen_?(t-attitude_received_)*1000:INFINITY;
    status.position_age_ms=estimate.initialized?(t-estimate.position_time)*1000:INFINITY;
    status.innovation_m=estimate.innovation;status.attitude_pair_skew_ms=pair_skew_;status.map_to_enu_yaw_rad=yaw_;
    status.accepted_uwb=estimate.accepted;status.rejected_uwb=estimate.rejected;
    status.duplicate_uwb=duplicate_uwb_;status.duplicate_flow=duplicate_flow_;status.flow_gaps=flow_gaps_;
    status_->publish(status);
    if (estimate.valid && estimate.position_time>last_published_time_) {
      nav_msgs::msg::Odometry odom;odom.header= status.header;
      odom.header.stamp=now()-rclcpp::Duration::from_seconds(std::max(0.,t-estimate.position_time));
      odom.child_frame_id=child_;odom.pose.pose.position=status.position;odom.pose.pose.orientation.w=1.;
      odom.pose.covariance[0]=estimate.covariance[0];odom.pose.covariance[1]=odom.pose.covariance[6]=estimate.covariance[1];
      odom.pose.covariance[7]=estimate.covariance[2];
      for(int index:{14,21,28,35}){odom.pose.covariance[index]=1e6;}
      odom.twist.twist.linear=status.velocity_map;
      for(int index:{0,7,14,21,28,35}){odom.twist.covariance[index]=1e6;}
      odometry_->publish(odom);
      geometry_msgs::msg::PoseWithCovarianceStamped pose;pose.header=odom.header;pose.pose=odom.pose;position_->publish(pose);
      geometry_msgs::msg::PoseStamped path_pose;path_pose.header=odom.header;path_pose.pose=odom.pose.pose;
      path_.header=odom.header;path_.poses.push_back(path_pose);
      if(path_.poses.size()>static_cast<size_t>(path_limit_)){path_.poses.erase(path_.poses.begin());}
      if(t-last_path_publish_>=1/path_rate_){path_output_->publish(path_);last_path_publish_=t;}
      last_published_time_=estimate.position_time;
    }
    visualization_msgs::msg::MarkerArray markers;visualization_msgs::msg::Marker point;
    point.header=status.header;point.ns="fusion_xy";point.id=0;point.type=point.SPHERE;
    point.action=estimate.initialized?point.ADD:point.DELETE;point.pose.position=status.position;point.pose.position.z=.3;
    point.pose.orientation.w=1;point.scale.x=point.scale.y=point.scale.z=.18;point.color.a=1;
    point.color.r=estimate.valid?(estimate.state=="DEAD_RECKONING"?1.:0.):.5;
    point.color.g=estimate.valid?.7:.5;point.color.b=estimate.valid?(estimate.state=="DEAD_RECKONING"?0.:.9):.5;markers.markers.push_back(point);
    auto text=point;text.id=1;text.type=text.TEXT_VIEW_FACING;text.pose.position.z=.65;text.scale.z=.16;
    text.text="Fusion XY: "+estimate.state+(estimate.valid?"":" (last known / invalid)");markers.markers.push_back(text);
    markers_->publish(markers);
  }
  LooseOdometryConfig config_;std::unique_ptr<LooseOdometry> filter_;AlignmentSession alignment_session_;
  std::string frame_,child_,alignment_reason_{"Align nose +X via /fusion/align_positive_x (or import navigation alignment)"};
  int attitude_timeout_,pair_skew_limit_,device_gap_limit_,system_,component_,path_limit_;
  double max_speed_,path_rate_,yaw_{0},pair_skew_{INFINITY},last_published_time_{-1},last_path_publish_{-INFINITY};
  bool use_navigation_alignment_,q_body_to_ned_,alignment_valid_{false},local_alignment_{false};
  bool pose_seen_{false},uwb_seen_{false},attitude_seen_{false},motion_seen_{false},motion_pending_{false};
  double pose_received_{0},uwb_received_{0},attitude_received_{0},motion_received_{0};
  int64_t pose_stamp_{0},processed_pose_stamp_{0};uint64_t duplicate_uwb_{0},duplicate_flow_{0},flow_gaps_{0};
  geometry_msgs::msg::PoseWithCovarianceStamped pose_;flapping_uav_msgs::msg::UwbStatus uwb_;
  flapping_uav_msgs::msg::AttitudeState attitude_;flapping_uav_msgs::msg::BodyMotionState motion_;
  std::deque<TimedAttitude> attitudes_;nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_;
  rclcpp::Publisher<flapping_uav_msgs::msg::FusionStatus>::SharedPtr status_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::UwbStatus>::SharedPtr uwb_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::AttitudeState>::SharedPtr attitude_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::BodyMotionState>::SharedPtr flow_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::HeadingAlignment>::SharedPtr alignment_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr align_service_,reset_service_;rclcpp::TimerBase::SharedPtr timer_;
};
rclcpp::Node::SharedPtr make_fusion_odometry_node(const rclcpp::NodeOptions & options)
{return std::make_shared<FusionOdometryNode>(options);}
}  // namespace flapping_uav_gcs
#ifndef FLAPPING_UAV_NO_MAIN
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try{rclcpp::spin(flapping_uav_gcs::make_fusion_odometry_node(rclcpp::NodeOptions()));}
  catch(const std::exception & e){RCLCPP_FATAL(rclcpp::get_logger("fusion_odometry_node"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
#endif
