#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <set>
#include <string>
#include <vector>
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/scene_geometry.hpp"
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/heading_alignment.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace flapping_uav_gcs
{
class SceneVisualizerNode : public rclcpp::Node
{
public:
  explicit SceneVisualizerNode(const rclcpp::NodeOptions & options)
  : Node("scene_visualizer_node", options)
  {
    map_ = parameter("map_frame_id", std::string("map"));
    uwb_frame_ = parameter("uwb_frame_id", std::string("uwb"));
    anchors_ready_ = parameter("anchor_coordinates_configured", false);
    anchor_z_known_ = parameter("anchor_z_configured", false);
    ids_ = parameter<std::vector<int64_t>>("anchor_ids", {0,1,2,3});
    anchors_ = parameter<std::vector<double>>("anchor_positions_m", std::vector<double>(12,0));
    uwb_yaw_ = parameter("uwb_to_map_yaw_rad", 0.0);
    uwb_y_sign_ = parameter("uwb_y_sign", 1.0);
    auto translation = parameter<std::vector<double>>("uwb_to_map_translation_m", {0,0,0});
    auto preview = parameter<std::vector<double>>("attitude_preview_origin_map_m", {-1,0,.5});
    aligned_ = parameter("map_to_reference_confirmed", false);
    attitude_confirmed_ = parameter("attitude_convention_confirmed", false);
    q_body_ned_ = parameter("quaternion_body_to_ned", true);
    map_enu_yaw_ = parameter("map_to_enu_yaw_rad", 0.0);
    alignment_session_.load(*this);
    project_xy_ = parameter("project_aircraft_to_xy", false);
    display_z_ = parameter("projected_aircraft_z_m", .25);
    timeout_ms_ = parameter("data_timeout_ms", 500);
    const double rate = parameter("publish_rate_hz", 20.0);
    if (map_.empty() || uwb_frame_.empty() || map_==uwb_frame_ || ids_.size()!=4 ||
      anchors_.size()!=12 || translation.size()!=3 || preview.size()!=3 ||
      !std::isfinite(uwb_yaw_) || !std::isfinite(map_enu_yaw_) || !std::isfinite(display_z_) ||
      !std::isfinite(uwb_y_sign_) || std::abs(uwb_y_sign_)!=1 ||
      timeout_ms_<=0 || !std::isfinite(rate) || rate<1 || rate>100)
    {throw std::invalid_argument("Invalid scene geometry/configuration");}
    std::set<int64_t> unique;
    for (auto id : ids_) {
      if (id<0 || id>255 || !unique.insert(id).second) {throw std::invalid_argument("Invalid/duplicate anchor ID");}
    }
    for (size_t i=0;i<3;++i) {translation_[i]=translation[i]; preview_[i]=preview[i];}
    if (!finite_vector(translation_) || !finite_vector(preview_)) {
      throw std::invalid_argument("Nonfinite scene origin");
    }
    if (anchors_ready_) {
      for (double v : anchors_) {if (!std::isfinite(v)) {throw std::invalid_argument("Nonfinite anchor coordinate");}}
      for (size_t i=0;i<4;++i) {for (size_t j=i+1;j<4;++j) {
        if (std::hypot(anchors_[3*i]-anchors_[3*j], anchors_[3*i+1]-anchors_[3*j+1],
          anchors_[3*i+2]-anchors_[3*j+2])<1e-6)
        {throw std::invalid_argument("Anchor coordinates overlap; replace placeholder values");}
      }}
    }
    output_ = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/scene_markers",
      rclcpp::QoS(1).reliable().transient_local());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now(); transform.header.frame_id=map_; transform.child_frame_id=uwb_frame_;
    transform.transform.translation.x=translation_[0]; transform.transform.translation.y=translation_[1];
    transform.transform.translation.z=translation_[2];
    transform.transform.rotation.z=std::sin(uwb_yaw_/2); transform.transform.rotation.w=std::cos(uwb_yaw_/2);
    static_tf_->sendTransform(transform);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/position", rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        pose_=*m; pose_time_=Clock::now(); pose_seen_=true;
      });
    status_sub_ = create_subscription<flapping_uav_msgs::msg::UwbStatus>("/uwb/status",10,
      [this](flapping_uav_msgs::msg::UwbStatus::ConstSharedPtr m) {status_=*m; status_time_=Clock::now(); status_seen_=true;});
    attitude_sub_ = create_subscription<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude",10,
      [this](flapping_uav_msgs::msg::AttitudeState::ConstSharedPtr m) {
        if (!attitude_seen_ || m->sample_time_us!=attitude_.sample_time_us ||
          m->reset_counter!=attitude_.reset_counter || m->system_id!=attitude_.system_id ||
          m->component_id!=attitude_.component_id || m->message_id!=attitude_.message_id)
        {attitude_time_=Clock::now(); attitude_seen_=true;}
        attitude_=*m;
      });
    motion_sub_ = create_subscription<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion",10,
      [this](flapping_uav_msgs::msg::BodyMotionState::ConstSharedPtr m) {motion_=*m; motion_time_=Clock::now(); motion_seen_=true;});
    alignment_sub_ = create_subscription<flapping_uav_msgs::msg::HeadingAlignment>(
      "/navigation/heading_alignment", rclcpp::QoS(1).reliable().transient_local(),
      [this](flapping_uav_msgs::msg::HeadingAlignment::ConstSharedPtr m) {
        if (m->header.frame_id != map_ || !std::isfinite(m->map_to_enu_yaw_rad)) {return;}
        aligned_ = m->valid;
        if (!m->valid) {return;}
        attitude_confirmed_ = true; q_body_ned_ = m->quaternion_body_to_ned;
        map_enu_yaw_ = m->map_to_enu_yaw_rad;
        AlignmentSessionConfig reference;
        reference.required = true; reference.device_time_us = m->device_time_us;
        reference.host_monotonic_ns = m->host_monotonic_ns;
        reference.host_boot_id = m->host_boot_id;
        reference.max_clock_skew_ms = m->max_clock_skew_ms;
        alignment_session_.set_reference(reference);
      });
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1/rate)),[this](){publish();});
    RCLCPP_INFO(get_logger(),"RViz scene: anchors=%s; full attitude overlay aligned=%s",
      anchors_ready_?"configured":"missing actual coordinates",aligned_&&attitude_confirmed_?"true":"false (separate preview)");
  }
private:
  using Clock=std::chrono::steady_clock;
  using Marker=visualization_msgs::msg::Marker;
  template<typename T> T parameter(const std::string & name,const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;return declare_parameter<T>(name,value,d);
  }
  bool fresh(bool seen,Clock::time_point when,const std_msgs::msg::Header & h) const
  {
    if (!seen || std::chrono::duration<double,std::milli>(Clock::now()-when).count()>timeout_ms_) {return false;}
    const int64_t stamp=rclcpp::Time(h.stamp).nanoseconds();
    const double ms=static_cast<double>(now().nanoseconds()-stamp)/1e6;
    return stamp>0 && ms>=-100 && ms<=timeout_ms_;
  }
  static geometry_msgs::msg::Point point(Velocity3 p)
  {geometry_msgs::msg::Point out;out.x=p[0];out.y=p[1];out.z=p[2];return out;}
  Marker marker(int id,int type,Velocity3 p,const std::string & frame)
  {
    Marker m;m.header.stamp=scene_stamp_;m.header.frame_id=frame;m.ns="debug_scene";
    m.id=id;m.type=type;m.action=Marker::ADD;m.pose.position=point(p);m.pose.orientation.w=1;
    m.color.a=1;return m;
  }
  void text(visualization_msgs::msg::MarkerArray & a,int id,Velocity3 p,const std::string & s,
    const std::string & frame,bool warning=false)
  {
    auto m=marker(id,Marker::TEXT_VIEW_FACING,p,frame);m.text=s;m.scale.z=.20;
    m.color.r=1;m.color.g=warning?.5:1;m.color.b=warning?0:1;a.markers.push_back(m);
  }
  void axes(visualization_msgs::msg::MarkerArray & a,int start,const std::string & frame,
    double length,const std::string & prefix)
  {
    const char * names[]={"X","Y","Z"};
    for (size_t i=0;i<3;++i) {
      Velocity3 end{{0,0,0}};end[i]=length;
      auto m=marker(start+static_cast<int>(2*i),Marker::ARROW,{{0,0,0}},frame);
      m.points={point({{0,0,0}}),point(end)};m.scale.x=.018;m.scale.y=.04;m.scale.z=.06;
      m.color.r=i==0?1:0;m.color.g=i==1?1:0;m.color.b=i==2?1:0;a.markers.push_back(m);
      end[i]+=.08;text(a,start+static_cast<int>(2*i+1),end,prefix+names[i],frame);
    }
  }
  void publish()
  {
    // Dynamic-frame markers must use the EXACT transform time. Stamping each
    // marker later than sendTransform causes future extrapolation; DELETEALL on
    // the next frame can then discard the still-waiting marker before it renders.
    scene_stamp_=now();
    visualization_msgs::msg::MarkerArray a;
    Marker clear;clear.action=Marker::DELETEALL;a.markers.push_back(clear);
    axes(a,0,uwb_frame_,1.0,uwb_y_sign_<0?"RH+":"UWB+");
    text(a,8,{{0,0,.08}},uwb_y_sign_<0?"UWB-RH0":"UWB0",uwb_frame_);
    if (uwb_y_sign_<0) {
      // A reflection is not a quaternion/TF rotation. Draw native +Y explicitly
      // while the published TF denotes the right-handed derived coordinate frame.
      auto native_y=marker(40,Marker::ARROW,{{0,0,0}},uwb_frame_);
      native_y.points={point({{0,0,0}}),point({{0,-1.3,0}})};
      native_y.scale.x=.018;native_y.scale.y=.04;native_y.scale.z=.06;
      native_y.color.g=.65;a.markers.push_back(native_y);
      text(a,41,{{0,-1.5,0}},"UWB+Y=-RH+Y",uwb_frame_);
    }
    if (anchors_ready_) {
      for (size_t i=0;i<4;++i) {
        Velocity3 p{{anchors_[3*i],uwb_y_sign_*anchors_[3*i+1],anchor_z_known_?anchors_[3*i+2]:0}};
        auto m=marker(100+static_cast<int>(i)*2,Marker::CUBE,p,uwb_frame_);
        m.scale.x=m.scale.y=m.scale.z=.16;m.color.r=1;m.color.g=.65;m.color.b=.1;a.markers.push_back(m);
        // Keep labels compact. Long space-separated captions overflow Foxy's
        // billboard text layout and can hide neighbouring anchors.
        p[0]+=.15;p[1]-=.3;p[2]+=.25;
        std::ostringstream label;label<<"A"<<ids_[i]<<"\n"<<std::fixed<<std::setprecision(3)
          <<anchors_[3*i]<<","<<anchors_[3*i+1]<<"(UWB)";
        if (!anchor_z_known_) {label<<"\nXY-only";}
        text(a,m.id+1,p,label.str(),uwb_frame_);
      }
    } else {text(a,9,{{0,0,.4}},"A0..A3\nCoordinatesMissing",uwb_frame_,true);}
    const auto & p=pose_.pose.pose.position;
    const bool position_ok=fresh(pose_seen_,pose_time_,pose_.header) &&
      fresh(status_seen_,status_time_,status_.header) && status_.position_valid &&
      status_.coordinates_confirmed && status_.header.frame_id==map_ &&
      pose_.header.frame_id==map_ && std::isfinite(p.x) && std::isfinite(p.y);
    const auto & q=attitude_.orientation;
    QuaternionWxyz source_q=attitude_.yaw_only?
      QuaternionWxyz{{std::cos(attitude_.yaw_rad/2),0,0,std::sin(attitude_.yaw_rad/2)}}:
      QuaternionWxyz{{q.w,q.x,q.y,q.z}};
    QuaternionWxyz rotation;
    const bool attitude_ok=fresh(attitude_seen_,attitude_time_,attitude_.header) && attitude_.valid &&
      attitude_.header.frame_id=="local_ned" && attitude_.body_frame_id=="base_link_frd" &&
      body_attitude_to_map(source_q,q_body_ned_,map_enu_yaw_,rotation);
    const bool session_ok=attitude_ok && alignment_session_.valid(attitude_.sample_time_us);
    const bool overlay=attitude_ok && position_ok && aligned_ && attitude_confirmed_ && session_ok;
    if (attitude_ok) {
      const std::string body_frame=overlay?"aircraft_body_frd":"aircraft_attitude_preview";
      Velocity3 origin=preview_;
      if (overlay) {origin={{p.x,p.y,project_xy_||!std::isfinite(p.z)?display_z_:p.z}};}
      geometry_msgs::msg::TransformStamped t;t.header.stamp=scene_stamp_;t.header.frame_id=map_;
      t.child_frame_id=body_frame;t.transform.translation.x=origin[0];t.transform.translation.y=origin[1];
      t.transform.translation.z=origin[2];t.transform.rotation.w=rotation[0];t.transform.rotation.x=rotation[1];
      t.transform.rotation.y=rotation[2];t.transform.rotation.z=rotation[3];tf_->sendTransform(t);
      auto fuselage=marker(1000,Marker::CUBE,{{0,0,0}},body_frame);
      fuselage.scale.x=.3;fuselage.scale.y=.065;fuselage.scale.z=.045;
      fuselage.color.r=overlay?.25:1;fuselage.color.g=.7;fuselage.color.b=overlay?1:0;a.markers.push_back(fuselage);
      auto wings=fuselage;wings.id=1001;wings.scale.x=.08;wings.scale.y=.55;wings.scale.z=.008;a.markers.push_back(wings);
      axes(a,1010,body_frame,.35,"Body+");
      origin[2]+=.45;
      std::string caption=overlay?"Aircraft@Tag":"UNALIGNED\nAttitudePreviewOnly";
      if (aligned_ && attitude_confirmed_ && !session_ok) {caption="SESSION-EXPIRED\nRecalibrateHeading";}
      normalize_attitude(source_q);
      if (!q_body_ned_) {for (size_t i=1;i<4;++i) {source_q[i]=-source_q[i];}}
      const double w=source_q[0],x=source_q[1],y=source_q[2],z=source_q[3];
      const double degrees=180/std::acos(-1);
      std::ostringstream angles;
      if(attitude_.yaw_only){angles<<std::fixed<<std::setprecision(1)<<"\nHeading only (level preview)\nYaw(deg):"<<attitude_.yaw_rad*degrees;}
      else {angles<<std::fixed<<std::setprecision(1)<<"\nFC-RPY(deg)\n"
        <<std::atan2(2*(w*x+y*z),1-2*(x*x+y*y))*degrees<<"/"
        <<std::asin(std::clamp(2*(w*y-z*x),-1.0,1.0))*degrees<<"/"
        <<std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))*degrees;}
      caption+=angles.str();
      if (overlay && (project_xy_||!std::isfinite(p.z))) {caption+="\nXY-projection";}
      if (fresh(motion_seen_,motion_time_,motion_.header) && motion_.height_valid) {
        caption+="\nHeight(m):"+std::to_string(motion_.height_up_m);
      }
      text(a,1050,origin,caption,map_,!overlay);
    } else {text(a,1050,preview_,"Waiting:ID31\nNoAircraftAttitude",map_,true);}
    output_->publish(a);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;diagnostics.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus d;d.name="scene_visualizer";d.hardware_id="RViz debug only";
    d.level=anchors_ready_&&overlay?d.OK:d.WARN;
    d.message=overlay?"Aircraft attitude aligned with UWB/map":"Scene incomplete; attitude preview only";
    auto add=[&](const std::string & key,bool value) {diagnostic_msgs::msg::KeyValue kv;kv.key=key;
      kv.value=value?"true":"false";d.values.push_back(kv);};
    add("anchor_coordinates_configured",anchors_ready_);add("uwb_position_fresh",position_ok);
    add("anchor_z_configured",anchor_z_known_);
    add("native_uwb_y_reflected",uwb_y_sign_<0);
    add("attitude_fresh",attitude_ok);add("attitude_overlay_aligned",overlay);
    add("alignment_session_valid",session_ok);
    diagnostics.status.push_back(d);diagnostics_->publish(diagnostics);
  }
  std::string map_,uwb_frame_;
  AlignmentSession alignment_session_;
  bool anchors_ready_,anchor_z_known_,aligned_,attitude_confirmed_,q_body_ned_,project_xy_;
  double uwb_yaw_,uwb_y_sign_,map_enu_yaw_,display_z_;int timeout_ms_;
  Velocity3 translation_{},preview_{};
  std::vector<int64_t> ids_;std::vector<double> anchors_;
  bool pose_seen_{false},status_seen_{false},attitude_seen_{false},motion_seen_{false};
  Clock::time_point pose_time_{},status_time_{},attitude_time_{},motion_time_{};
  rclcpp::Time scene_stamp_{0,0,RCL_ROS_TIME};
  geometry_msgs::msg::PoseWithCovarianceStamped pose_;
  flapping_uav_msgs::msg::UwbStatus status_;flapping_uav_msgs::msg::AttitudeState attitude_;
  flapping_uav_msgs::msg::BodyMotionState motion_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::UwbStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::AttitudeState>::SharedPtr attitude_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::BodyMotionState>::SharedPtr motion_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::HeadingAlignment>::SharedPtr alignment_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
rclcpp::Node::SharedPtr make_scene_visualizer_node(const rclcpp::NodeOptions & options)
{return std::make_shared<SceneVisualizerNode>(options);}
}  // namespace flapping_uav_gcs
#ifndef FLAPPING_UAV_NO_MAIN
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(flapping_uav_gcs::make_scene_visualizer_node(rclcpp::NodeOptions()));}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("scene_visualizer_node"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
#endif
