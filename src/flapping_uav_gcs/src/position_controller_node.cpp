#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <sstream>
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "flapping_uav_msgs/msg/heading_alignment.hpp"

#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/position_controller.hpp"
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/mavlink_frame.hpp"
#include "flapping_uav_msgs/msg/navigation_status.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace flapping_uav_gcs
{
class PositionControllerNode : public rclcpp::Node
{
public:
  explicit PositionControllerNode(const rclcpp::NodeOptions & options)
  : Node("position_controller_node", options)
  {
    map_frame_ = parameter("map_frame_id", std::string("map"));
    feedback_mode_ = parameter("feedback_velocity_mode", std::string("unconfirmed"));
    alignment_ = parameter("map_to_reference_confirmed", false);
    attitude_confirmed_ = parameter("attitude_convention_confirmed", false);
    feedback_confirmed_ = parameter("feedback_convention_confirmed", false);
    zero_vertical_ = parameter("assume_zero_world_vertical_speed", false);
    q_body_to_ned_ = parameter("quaternion_body_to_ned", true);
    yaw_ = parameter("map_to_enu_yaw_rad", 0.0);
    alignment_session_.load(*this);
    runtime_start_ = parameter("require_start_here", false);
    direct_body_ = parameter("publish_body_setpoint", false);
    invalid_velocity_as_zero_ = parameter("invalid_velocity_as_zero", false);
    if (runtime_start_) {alignment_ = false;}
    if (direct_body_ && feedback_mode_ != "heading_xy") {
      throw std::invalid_argument("Direct body output requires heading_xy feedback");
    }
    position_timeout_ = parameter("position_timeout_ms", 500);
    feedback_timeout_ = parameter("feedback_timeout_ms", 200);
    attitude_timeout_ = parameter("attitude_timeout_ms", 200);
    heartbeat_timeout_ = parameter("heartbeat_timeout_ms", 3000);
    heartbeat_required_ = parameter("heartbeat_required", true);
    recovery_grace_ms_ = parameter("input_recovery_grace_ms", 0);
    if (recovery_grace_ms_ < 0) {throw std::invalid_argument("Negative recovery grace");}
    target_system_ = parameter("target_system", 1);
    target_component_ = parameter("target_component", 1);
    const double rate = parameter("publish_rate_hz", 50.0);
    config_.kp = parameter("kp_xy", 0.0); config_.kd = parameter("kd_xy", 0.0);
    config_.max_speed = parameter("max_speed_mps", 0.0);
    config_.max_acceleration = parameter("max_acceleration_mps2", 0.0);
    config_.position_tolerance = parameter("arrival_position_tolerance_m", 0.0);
    config_.speed_tolerance = parameter("arrival_speed_tolerance_mps", 0.0);
    config_.dwell_s = parameter("arrival_dwell_s", 0.0);
    config_.continuous_hold = parameter("continuous_hold", false);
    controller_ = std::make_unique<PositionController>(config_);
    if (map_frame_.empty() || !std::isfinite(yaw_) || !std::isfinite(rate) ||
      rate < 1 || rate > 200 || position_timeout_ <= 0 || feedback_timeout_ <= 0 ||
      attitude_timeout_ <= 0 || heartbeat_timeout_ <= 0 ||
      target_system_ < 1 || target_system_ > 255 || target_component_ < 0 ||
      target_component_ > 255 || (feedback_mode_ != "unconfirmed" &&
      feedback_mode_ != "heading_xy" && feedback_mode_ != "body_frd_xy"))
    {throw std::invalid_argument("Invalid position controller configuration");}
    period_ = 1.0 / rate;
    velocity_output_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/navigation/velocity_setpoint_map", 10);
    status_output_ = create_publisher<flapping_uav_msgs::msg::NavigationStatus>(
      "/navigation/status", 10);
    goal_output_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/navigation/goal_markers", rclcpp::QoS(1).transient_local());
    if (direct_body_) {
      body_output_ = create_publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
        "/fc/velocity_setpoint_body", 10);
    }
    alignment_output_ = create_publisher<flapping_uav_msgs::msg::HeadingAlignment>(
      "/navigation/heading_alignment", rclcpp::QoS(1).reliable().transient_local());
    if (runtime_start_) {publish_alignment(false, "Waiting for +X alignment and start_here");}
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/position", rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        const int64_t stamp = rclcpp::Time(m->header.stamp).nanoseconds();
        if (pose_seen_ && stamp <= pose_stamp_) {return;}
        pose_ = *m; pose_stamp_ = stamp; pose_seen_ = stamp > 0;
        pose_time_ = Clock::now();
      });
    uwb_sub_ = create_subscription<flapping_uav_msgs::msg::UwbStatus>(
      "/uwb/status", 10, [this](flapping_uav_msgs::msg::UwbStatus::ConstSharedPtr m) {
        uwb_ = *m; uwb_time_ = Clock::now(); uwb_seen_ = true;
      });
    motion_sub_ = create_subscription<flapping_uav_msgs::msg::BodyMotionState>(
      "/fc/body_motion", rclcpp::QoS(1),
      [this](flapping_uav_msgs::msg::BodyMotionState::ConstSharedPtr m) {
        if (m->system_id != target_system_ || m->component_id != target_component_) {return;}
        if (motion_seen_ && m->sample_time_us < motion_.sample_time_us) {
          stop("FAULT", "ID250 device clock reset; submit a new goal");
        }
        if (!motion_seen_ || m->sample_time_us != motion_.sample_time_us) {
          motion_time_ = Clock::now(); motion_seen_ = true;
        }
        motion_ = *m;
      });
    attitude_sub_ = create_subscription<flapping_uav_msgs::msg::AttitudeState>(
      "/fc/attitude", rclcpp::QoS(1),
      [this](flapping_uav_msgs::msg::AttitudeState::ConstSharedPtr m) {
        if (m->system_id != target_system_ || m->component_id != target_component_) {return;}
        if (attitude_seen_ && (m->sample_time_us < attitude_.sample_time_us ||
          m->reset_counter != attitude_.reset_counter))
        {alignment_ = false;
          alignment_reason_ = "Attitude reference reset: device_us " +
            std::to_string(attitude_.sample_time_us) + " -> " +
            std::to_string(m->sample_time_us) + "; point nose to +X and call start_here again";
          publish_alignment(false, alignment_reason_);
          RCLCPP_WARN(get_logger(), "%s", alignment_reason_.c_str());
          stop("FAULT", alignment_reason_);}
        if (!attitude_seen_ || m->sample_time_us != attitude_.sample_time_us ||
          m->reset_counter != attitude_.reset_counter)
        {attitude_time_ = Clock::now(); attitude_seen_ = true;}
        attitude_ = *m;
      });
    heartbeat_sub_ = create_subscription<flapping_uav_msgs::msg::MavlinkFrame>(
      "/fc/mavlink_rx", 100, [this](flapping_uav_msgs::msg::MavlinkFrame::ConstSharedPtr m) {
        if (m->message_id == 0 && m->system_id == target_system_ &&
          m->component_id == target_component_ && header_fresh(m->header, heartbeat_timeout_))
        {heartbeat_time_ = Clock::now(); heartbeat_seen_ = true;}
      });
    auto goal_callback = [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {goal(*m);};
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, goal_callback);
    navigation_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/navigation/goal", 10, goal_callback);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/navigation/enable", [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!request->data) {
          enabled_ = false; stop("DISABLED", "Controller disabled; goal cleared");
          response->success = true; response->message = reason_; return;
        }
        Velocity3 measured;
        const auto why = readiness(measured);
        response->success = why.empty();
        if (why.empty()) {enabled_ = true; state_ = "IDLE"; reason_ = "Enabled; waiting for goal";}
        response->message = why.empty() ? reason_ : why;
      });
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/navigation/start_here", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start_here(*response);});
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/navigation/cancel_goal", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        stop("CANCELLED", "Goal cancelled"); response->success = true; response->message = reason_;
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_)),
      [this]() {tick();});
    RCLCPP_INFO(get_logger(), "XY P/PD disabled; heartbeat=%s; ID250 z is height only",
      heartbeat_required_ ? "required" : "diagnostic (fresh FC attitude/velocity still required)");
  }
private:
  using Clock = std::chrono::steady_clock;
  template<typename T> T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  static double age(bool seen, Clock::time_point when)
  {
    return seen ? std::chrono::duration<double, std::milli>(Clock::now() - when).count() :
           std::numeric_limits<double>::infinity();
  }
  bool header_fresh(const std_msgs::msg::Header & header, int timeout_ms) const
  {
    const int64_t stamp = rclcpp::Time(header.stamp).nanoseconds();
    if (stamp <= 0) {return false;}
    const double delta_ms = static_cast<double>(now().nanoseconds() - stamp) / 1e6;
    // All typed inputs are stamped on this computer by their receiving nodes.
    return delta_ms >= -100 && delta_ms <= timeout_ms;
  }
  bool feedback_assumed_zero() const
  {
    return invalid_velocity_as_zero_ &&
      (!motion_.velocity_valid || !std::isfinite(motion_.forward_velocity_mps) ||
       !std::isfinite(motion_.right_velocity_mps));
  }
  std::string readiness(
    Velocity3 & measured, bool calibrating = false, bool skip_feedback = false,
    bool * transient_failure = nullptr) const
  {
    if (transient_failure) {*transient_failure = false;}
    const auto temporary = [transient_failure](const std::string & why) {
        if (transient_failure) {*transient_failure = true;}
        return why;
      };
    if (!config_.configured()) {return "Set commissioned gains, limits and arrival thresholds";}
    if ((!alignment_ && !calibrating) || !attitude_confirmed_) {
      if (!alignment_ && !alignment_reason_.empty()) {return alignment_reason_;}
      return runtime_start_ ? "Point nose to map +X, then call /navigation/start_here" :
             "Map/attitude conventions unconfirmed";
    }
    if (!calibrating && (!feedback_confirmed_ || feedback_mode_ == "unconfirmed")) {
      return "ID250 velocity frame unconfirmed";
    }
    if (heartbeat_required_ && age(heartbeat_seen_, heartbeat_time_) > heartbeat_timeout_) {
      return temporary("Heartbeat missing/stale");
    }
    if (!uwb_seen_ || age(true, uwb_time_) > position_timeout_ ||
      !uwb_.position_valid || !uwb_.coordinates_confirmed || uwb_.header.frame_id != map_frame_ ||
      !header_fresh(uwb_.header, position_timeout_))
    {return temporary("UWB status invalid/stale or coordinates unconfirmed");}
    const auto & p = pose_.pose.pose.position;
    if (age(pose_seen_, pose_time_) > position_timeout_ ||
      pose_.header.frame_id != map_frame_ || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !header_fresh(pose_.header, position_timeout_))
    {return temporary("UWB XY position missing/stale/invalid");}
    if (!attitude_seen_) {return temporary("ID31 attitude missing");}
    if (age(true, attitude_time_) > attitude_timeout_ ||
      !header_fresh(attitude_.header, attitude_timeout_)) {
      const auto header_age = static_cast<double>(now().nanoseconds() -
        rclcpp::Time(attitude_.header.stamp).nanoseconds()) / 1e6;
      return temporary("ID31 attitude stale: sample_age_ms=" +
        std::to_string(age(true, attitude_time_)) + ", header_age_ms=" +
        std::to_string(header_age) + ", limit_ms=" + std::to_string(attitude_timeout_) +
        ", device_us=" + std::to_string(attitude_.sample_time_us));
    }
    if (attitude_.header.frame_id != "local_ned" || attitude_.body_frame_id != "base_link_frd") {
      return "ID31 attitude frame mismatch";
    }
    if (!attitude_.valid) {
      const auto & q = attitude_.orientation;
      return temporary("ID31 attitude invalid: " + attitude_.reason + "; q(wxyz)=" +
        std::to_string(q.w) + "," + std::to_string(q.x) + "," +
        std::to_string(q.y) + "," + std::to_string(q.z));
    }
    if (!calibrating && !alignment_session_.valid(attitude_.sample_time_us)) {
      return "Heading alignment expired: FC/PC reboot or clock reference changed; recalibrate";
    }
    const auto & q = attitude_.orientation;
    // Calibration only needs current pose, heartbeat and a usable attitude.
    // In particular, a ground-level optical-flow NaN must not prevent +X capture.
    if (calibrating || skip_feedback) {
      if (!map_velocity_to_heading({{0, 0, 0}}, yaw_, {{q.w, q.x, q.y, q.z}},
        q_body_to_ned_, measured)) {return temporary("Cannot extract heading from attitude");}
      return {};
    }
    if (age(motion_seen_, motion_time_) > feedback_timeout_ ||
      !header_fresh(motion_.header, feedback_timeout_))
    {return temporary("ID250 velocity missing/stale (limit " +
      std::to_string(feedback_timeout_) + " ms)");}
    if (motion_.header.frame_id != "body_xy") {return "ID250 velocity frame mismatch";}
    if (feedback_assumed_zero()) {measured = {{0, 0, 0}}; return {};}
    if (!motion_.velocity_valid) {
      return temporary("ID250 XY velocity invalid; zero fallback disabled");
    }
    if (!feedback_xy_to_map(motion_.forward_velocity_mps, motion_.right_velocity_mps, yaw_,
        {{q.w, q.x, q.y, q.z}}, q_body_to_ned_, feedback_mode_ == "heading_xy",
        zero_vertical_, measured))
    {return temporary("Cannot transform XY feedback: quaternion or missing vertical velocity assumption");}
    return {};
  }
  void publish_alignment(bool valid, const std::string & reason)
  {
    flapping_uav_msgs::msg::HeadingAlignment m;
    m.header.stamp = now(); m.header.frame_id = map_frame_;
    m.valid = valid; m.reason = reason; m.map_to_enu_yaw_rad = yaw_;
    m.quaternion_body_to_ned = q_body_to_ned_;
    const auto & reference = alignment_session_.reference();
    m.device_time_us = reference.device_time_us;
    m.host_monotonic_ns = reference.host_monotonic_ns;
    m.host_boot_id = reference.host_boot_id;
    m.max_clock_skew_ms = reference.max_clock_skew_ms;
    alignment_output_->publish(m);
  }
  void start_here(std_srvs::srv::Trigger::Response & response)
  {
    Velocity3 measured;
    const auto why = readiness(measured, true);
    if (!why.empty()) {response.success = false; response.message = why; return;}
    const auto & orientation = attitude_.orientation;
    QuaternionWxyz q{{orientation.w, orientation.x, orientation.y, orientation.z}};
    if (!normalize_attitude(q)) {
      response.success = false; response.message = "Invalid quaternion"; return;
    }
    if (!q_body_to_ned_) {for (size_t i = 1; i < 4; ++i) {q[i] = -q[i];}}
    const auto forward = rotate_vector(q, {{1, 0, 0}});
    if (std::hypot(forward[0], forward[1]) < 1e-3 ||
      !alignment_session_.bind(attitude_.sample_time_us)) {
      response.success = false; response.message = "Cannot capture heading/session"; return;
    }
    const double alpha = std::acos(-1.)/2 - std::atan2(forward[1], forward[0]);
    yaw_ = std::atan2(std::sin(alpha), std::cos(alpha)); alignment_ = true;
    alignment_reason_.clear();
    controller_->stop(); recovery_active_ = false;
    geometry_msgs::msg::PoseStamped target;
    target.header = pose_.header; target.pose = pose_.pose.pose;
    // Capture the target without requiring a usable optical-flow sample.
    controller_->set_goal({{target.pose.position.x, target.pose.position.y}});
    publish_goal_marker(target);
    enabled_ = true;
    waiting_for_feedback_ = !readiness(measured).empty();
    state_ = waiting_for_feedback_ ? "WAITING_FEEDBACK" : "ACTIVE";
    std::ostringstream text;
    text << "Captured +X heading; holding map XY=(" << target.pose.position.x << ", "
         << target.pose.position.y << "); yaw offset=" << yaw_ << " rad";
    if (waiting_for_feedback_) {text << "; waiting for a fresh ID250 packet";}
    else if (feedback_assumed_zero()) {text << "; ID250 invalid: assuming zero XY velocity";}
    reason_ = text.str(); response.success = true; response.message = reason_;
    publish_alignment(true, reason_);
    RCLCPP_INFO(get_logger(), "%s", reason_.c_str());
  }
  void stop(const std::string & state, const std::string & reason)
  {
    if (state == "FAULT" && state_ != "FAULT") {
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    }
    controller_->stop(); waiting_for_feedback_ = false; recovery_active_ = false;
    state_ = state; reason_ = reason;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker m; m.action = m.DELETEALL;
    array.markers.push_back(m); goal_output_->publish(array);
  }
  void goal(const geometry_msgs::msg::PoseStamped & message)
  {
    Velocity3 measured;
    const auto why = readiness(measured);
    if (!why.empty() || message.header.frame_id != map_frame_ ||
      !std::isfinite(message.pose.position.x) || !std::isfinite(message.pose.position.y))
    {
      RCLCPP_WARN(get_logger(), "Goal rejected: %s", why.empty() ? "Wrong frame/nonfinite XY" : why.c_str());
      return;
    }
    controller_->set_goal({{message.pose.position.x, message.pose.position.y}});
    recovery_active_ = false;
    state_ = enabled_ ? "ACTIVE" : "READY"; reason_ = "XY goal accepted; yaw and z ignored";
    publish_goal_marker(message);
  }
  void publish_goal_marker(const geometry_msgs::msg::PoseStamped & message)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_; m.header.stamp = now(); m.ns = "navigation_goal";
    m.type = m.CYLINDER; m.action = m.ADD; m.pose.position = message.pose.position;
    m.pose.position.z = 0; m.pose.orientation.w = 1;
    m.scale.x = m.scale.y = 0.2; m.scale.z = 0.02;
    m.color.r = 1; m.color.g = 0.6; m.color.a = 1;
    array.markers.push_back(m); goal_output_->publish(array);
  }
  void tick()
  {
    const auto current = Clock::now();
    const double dt = tick_seen_ ? std::chrono::duration<double>(current - tick_time_).count() : period_;
    tick_time_ = current; tick_seen_ = true;
    Velocity3 measured{{0, 0, 0}};
    bool transient_failure = false;
    const auto why = readiness(measured, false, false, &transient_failure);
    PositionControlResult result;
    // Even a first valid packet received after the deadline cannot resume a goal.
    if (recovery_active_ && std::chrono::duration<double, std::milli>(
      current - recovery_started_).count() >= recovery_grace_ms_) {
      stop("FAULT", "Input recovery window expired (" + std::to_string(recovery_grace_ms_) +
        " ms): " + recovery_reason_ + "; goal cleared, call start_here again");
    }
    if (enabled_ && !why.empty() && state_ != "FAULT") {
      Velocity3 unused;
      if (waiting_for_feedback_ && controller_->has_goal() &&
        readiness(unused, false, true).empty()) {
        recovery_active_ = false;
        state_ = "WAITING_FEEDBACK"; reason_ = why + "; heading/goal captured; output zero";
      } else if (transient_failure && recovery_grace_ms_ > 0 && controller_->has_goal()) {
        if (!recovery_active_) {
          recovery_active_ = true; recovery_started_ = current; recovery_reason_ = why;
          RCLCPP_WARN(get_logger(), "Input recovery started (%d ms); output zero; %s",
            recovery_grace_ms_, why.c_str());
        }
        controller_->pause(); state_ = "INPUT_HOLD";
        const auto remaining = recovery_grace_ms_ - static_cast<int>(
          std::chrono::duration<double, std::milli>(current - recovery_started_).count());
        reason_ = "Input recovery: " + why + "; output zero; retaining goal; remaining_ms=" +
          std::to_string(remaining);
      } else {stop("FAULT", why + "; goal cleared, call start_here again");}
    }
    if (enabled_ && why.empty() && controller_->has_goal()) {
      if (recovery_active_) {
        RCLCPP_INFO(get_logger(), "Input recovered within grace; resuming original XY goal from zero");
        recovery_active_ = false;
      }
      waiting_for_feedback_ = false;
      const auto & p = pose_.pose.pose.position;
      result = controller_->step({{p.x, p.y}}, {{measured[0], measured[1]}}, dt);
      if (!result.valid) {stop("FAULT", "Control arithmetic invalid");}
      else {state_ = result.reached ? "REACHED" : "ACTIVE";
        reason_ = result.reached ? "Position and actual speed held within tolerances" : "XY P/PD active";}
    }
    const bool assumed_zero = why.empty() && feedback_assumed_zero();
    if (enabled_ && result.valid && assumed_zero) {
      reason_ += "; ID250 invalid: assuming zero XY velocity";
    }
    if (enabled_ && result.valid && !heartbeat_required_ &&
      age(heartbeat_seen_, heartbeat_time_) > heartbeat_timeout_) {
      reason_ += "; heartbeat stale (diagnostic); fresh FC telemetry available";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "Heartbeat stale; continuing with fresh verified-source ID31/ID250 (heartbeat_required=false)");
    }
    std::string display_reason = reason_;
    if (why.empty() && state_ == "FAULT" && !controller_->has_goal()) {
      display_reason = "Inputs recovered; previous fault remains latched: " + reason_;
    }
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now(); output.header.frame_id = map_frame_;
    output.twist.linear.x = result.velocity[0]; output.twist.linear.y = result.velocity[1];
    // UWB z and ID250 height never enter the world vertical velocity command.
    velocity_output_->publish(output);
    if (body_output_) {
      flapping_uav_msgs::msg::BodyVelocitySetpoint body;
      body.header.stamp = output.header.stamp; body.header.frame_id = "base_link_heading_frd";
      body.command_age_ms = 0; body.attitude_age_ms = age(attitude_seen_, attitude_time_);
      const auto & q = attitude_.orientation;
      Velocity3 v;
      body.valid = enabled_ && why.empty() && result.valid &&
        map_velocity_to_heading({{result.velocity[0], result.velocity[1], 0}}, yaw_,
          {{q.w, q.x, q.y, q.z}}, q_body_to_ned_, v);
      body.reason = state_ == "INPUT_HOLD" ? reason_ : (why.empty() ? display_reason : why);
      if (body.valid) {body.velocity.x = v[0]; body.velocity.y = v[1];}
      body_output_->publish(body);
    }
    flapping_uav_msgs::msg::NavigationStatus status;
    status.header = output.header; status.enabled = enabled_; status.inputs_valid = why.empty();
    status.state = state_; status.reason = !enabled_ && !why.empty() ? why : display_reason;
    status.goal_active = controller_->has_goal(); status.reached = result.reached;
    status.goal.x = controller_->goal()[0]; status.goal.y = controller_->goal()[1];
    status.distance_m = result.distance;
    status.feedback_assumed_zero = assumed_zero;
    // An assumption must never be reported as an actually measured zero speed.
    status.actual_speed_mps = why.empty() && !assumed_zero ? std::hypot(measured[0], measured[1]) :
      std::numeric_limits<double>::quiet_NaN();
    status.position_age_ms = age(pose_seen_, pose_time_);
    status.feedback_age_ms = age(motion_seen_, motion_time_);
    status.attitude_age_ms = age(attitude_seen_, attitude_time_);
    status.heartbeat_age_ms = age(heartbeat_seen_, heartbeat_time_);
    status.velocity_map = output.twist.linear;
    status_output_->publish(status);
  }
  PositionControlConfig config_;
  AlignmentSession alignment_session_;
  std::unique_ptr<PositionController> controller_;
  std::string map_frame_, feedback_mode_, state_{"DISABLED"}, reason_{"Controller disabled"};
  std::string alignment_reason_;
  bool runtime_start_{false}, direct_body_{false}, invalid_velocity_as_zero_{false};
  bool waiting_for_feedback_{false}, heartbeat_required_{true}, recovery_active_{false};
  int recovery_grace_ms_{0};
  std::string recovery_reason_;
  Clock::time_point recovery_started_{};
  bool enabled_{false}, alignment_, attitude_confirmed_, feedback_confirmed_, zero_vertical_, q_body_to_ned_;
  bool pose_seen_{false}, uwb_seen_{false}, motion_seen_{false}, attitude_seen_{false};
  bool heartbeat_seen_{false}, tick_seen_{false};
  int position_timeout_, feedback_timeout_, attitude_timeout_, heartbeat_timeout_, target_system_, target_component_;
  int64_t pose_stamp_{0};
  double yaw_, period_;
  Clock::time_point pose_time_{}, uwb_time_{}, motion_time_{}, attitude_time_{}, heartbeat_time_{}, tick_time_{};
  geometry_msgs::msg::PoseWithCovarianceStamped pose_;
  flapping_uav_msgs::msg::UwbStatus uwb_;
  flapping_uav_msgs::msg::BodyMotionState motion_;
  flapping_uav_msgs::msg::AttitudeState attitude_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_output_;
  rclcpp::Publisher<flapping_uav_msgs::msg::NavigationStatus>::SharedPtr status_output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_output_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::UwbStatus>::SharedPtr uwb_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::BodyMotionState>::SharedPtr motion_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::AttitudeState>::SharedPtr attitude_sub_;
  rclcpp::Subscription<flapping_uav_msgs::msg::MavlinkFrame>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, navigation_goal_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_, start_service_;
  rclcpp::Publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>::SharedPtr body_output_;
  rclcpp::Publisher<flapping_uav_msgs::msg::HeadingAlignment>::SharedPtr alignment_output_;
  rclcpp::TimerBase::SharedPtr timer_;
};
rclcpp::Node::SharedPtr make_position_controller_node(const rclcpp::NodeOptions & options)
{return std::make_shared<PositionControllerNode>(options);}
}  // namespace flapping_uav_gcs

#ifndef FLAPPING_UAV_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(flapping_uav_gcs::make_position_controller_node(rclcpp::NodeOptions()));}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("position_controller_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
#endif
