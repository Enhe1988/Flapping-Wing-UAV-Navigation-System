#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace flapping_uav_gcs
{
class VelocityTransformNode : public rclcpp::Node
{
public:
  explicit VelocityTransformNode(const rclcpp::NodeOptions & options)
  : Node("velocity_transform_node", options)
  {
    map_frame_ = parameter("map_frame_id", std::string("map"));
    reference_frame_ = parameter("attitude_reference_frame", std::string("local_ned"));
    rotation_mode_ = parameter("rotation_mode", std::string("full_attitude"));
    body_frame_ = parameter("body_frame_id", rotation_mode_ == "heading_only" ?
      std::string("base_link_heading_frd") : std::string("base_link_frd"));
    map_confirmed_ = parameter("map_to_reference_confirmed", false);
    attitude_confirmed_ = parameter("attitude_convention_confirmed", false);
    q_body_to_ned_ = parameter("quaternion_body_to_ned", true);
    yaw_ = parameter("map_to_enu_yaw_rad", 0.0);
    alignment_session_.load(*this);
    command_timeout_ms_ = parameter("command_timeout_ms", 200);
    attitude_timeout_ms_ = parameter("attitude_timeout_ms", 200);
    max_speed_ = parameter("max_speed_mps", 1.0);
    const double rate = parameter("publish_rate_hz", 50.0);
    if (map_frame_.empty() || reference_frame_.empty() || body_frame_.empty() ||
      !std::isfinite(yaw_) || !std::isfinite(max_speed_) || max_speed_ <= 0 ||
      command_timeout_ms_ <= 0 || attitude_timeout_ms_ <= 0 ||
      !std::isfinite(rate) || rate < 1 || rate > 200 ||
      (rotation_mode_ != "full_attitude" && rotation_mode_ != "heading_only") ||
      (rotation_mode_ == "heading_only" && body_frame_ != "base_link_heading_frd"))
    {throw std::invalid_argument("Invalid velocity transform configuration");}
    output_ = create_publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
      "/fc/velocity_setpoint_body", 10);
    command_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/navigation/velocity_setpoint_map", rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
        command_value_ = *message; command_time_ = Clock::now(); command_seen_ = true;
      });
    attitude_ = create_subscription<flapping_uav_msgs::msg::AttitudeState>(
      "/fc/attitude", rclcpp::QoS(1),
      [this](flapping_uav_msgs::msg::AttitudeState::ConstSharedPtr message) {
        attitude_value_ = *message;
        const auto key = std::make_tuple(
          message->system_id, message->component_id,
          message->message_id, message->sample_time_us, message->reset_counter);
        // Repeating device timestamps must not keep old attitude measurements fresh.
        if (!last_sample_ || key != *last_sample_) {
          last_sample_ = key; attitude_time_ = Clock::now(); attitude_seen_ = true;
        }
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1 / rate)), [this]() {publish();});
    RCLCPP_INFO(
      get_logger(), "map velocity -> %s (%s); map verified=%s, attitude verified=%s",
      body_frame_.c_str(), rotation_mode_.c_str(),
      map_confirmed_ ? "true" : "false", attitude_confirmed_ ? "true" : "false");
  }

private:
  using Clock = std::chrono::steady_clock;
  template<typename T>
  T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  void publish()
  {
    flapping_uav_msgs::msg::BodyVelocitySetpoint output;
    output.header.stamp = now(); output.header.frame_id = body_frame_;
    const auto current = Clock::now();
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    output.command_age_ms = command_seen_ ?
      std::chrono::duration<double, std::milli>(current - command_time_).count() : unknown;
    output.attitude_age_ms = attitude_seen_ ?
      std::chrono::duration<double, std::milli>(current - attitude_time_).count() : unknown;
    const auto reject = [this, &output](const std::string & reason) {
        output.reason = reason; output.valid = false; output_->publish(output);
      };
    if (!map_confirmed_) {reject("Map/ENU/NED alignment unconfirmed"); return;}
    if (!attitude_confirmed_) {reject("Attitude direction and body axes unconfirmed"); return;}
    if (!command_seen_ || output.command_age_ms > command_timeout_ms_) {
      reject("World velocity command missing/stale"); return;
    }
    if (!attitude_seen_ || output.attitude_age_ms > attitude_timeout_ms_) {
      reject("Attitude missing/stale or device time not advancing"); return;
    }
    if (!attitude_value_.valid) {reject("Invalid attitude: " + attitude_value_.reason); return;}
    if (!alignment_session_.valid(attitude_value_.sample_time_us)) {
      reject("Heading alignment expired: FC/PC reboot or clock reference changed"); return;
    }
    if (command_value_.header.frame_id != map_frame_ ||
      attitude_value_.header.frame_id != reference_frame_ ||
      attitude_value_.body_frame_id != "base_link_frd")
    {reject("Input coordinate frame mismatch"); return;}
    const auto & linear = command_value_.twist.linear;
    const auto & angular = command_value_.twist.angular;
    const Velocity3 v{{linear.x, linear.y, linear.z}};
    if (!finite_vector(v) || !std::isfinite(angular.x) || !std::isfinite(angular.y) ||
      !std::isfinite(angular.z) || angular.x != 0 || angular.y != 0 || angular.z != 0 ||
      std::hypot(v[0], v[1], v[2]) > max_speed_)
    {reject("Velocity nonfinite, angular command unsupported or speed limit exceeded"); return;}
    const auto & orientation = attitude_value_.orientation;
    Velocity3 body;
    const QuaternionWxyz q{{orientation.w, orientation.x, orientation.y, orientation.z}};
    const bool transformed = rotation_mode_ == "heading_only" ?
      map_velocity_to_heading(v, yaw_, q, q_body_to_ned_, body) :
      map_velocity_to_body(v, yaw_, q, q_body_to_ned_, body);
    if (!transformed)
    {reject("Quaternion/velocity invalid"); return;}
    output.velocity.x = body[0]; output.velocity.y = body[1]; output.velocity.z = body[2];
    output.valid = true; output.reason = rotation_mode_ == "heading_only" ?
      "valid horizontal heading XY velocity" : "valid FRD velocity";
    output_->publish(output);
  }
  std::string map_frame_, reference_frame_, body_frame_, rotation_mode_;
  AlignmentSession alignment_session_;
  bool map_confirmed_, attitude_confirmed_, q_body_to_ned_;
  bool command_seen_{false}, attitude_seen_{false};
  double yaw_, max_speed_;
  int command_timeout_ms_, attitude_timeout_ms_;
  Clock::time_point command_time_{}, attitude_time_{};
  std::optional<std::tuple<uint8_t, uint8_t, uint32_t, uint64_t, uint8_t>> last_sample_;
  geometry_msgs::msg::TwistStamped command_value_;
  flapping_uav_msgs::msg::AttitudeState attitude_value_;
  rclcpp::Publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>::SharedPtr output_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_;
  rclcpp::Subscription<flapping_uav_msgs::msg::AttitudeState>::SharedPtr attitude_;
  rclcpp::TimerBase::SharedPtr timer_;
};
rclcpp::Node::SharedPtr make_velocity_transform_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<VelocityTransformNode>(options);
}
}  // namespace flapping_uav_gcs

#ifndef FLAPPING_UAV_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(flapping_uav_gcs::make_velocity_transform_node(rclcpp::NodeOptions()));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("velocity_transform_node"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
#endif
