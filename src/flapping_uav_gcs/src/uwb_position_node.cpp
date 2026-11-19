#include "flapping_uav_gcs/uwb_position_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace flapping_uav_gcs
{
double monotonic_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

UwbPositionNode::UwbPositionNode(const rclcpp::NodeOptions & options)
: Node("uwb_position_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Commissioning/startup parameter; restart to change";
  UwbConfig config;
  config.tag_id = declare_parameter<int>("tag_id", 0, descriptor);
  config.tag_role = declare_parameter<int>("tag_role", -1, descriptor);
  config.timeout_s = declare_parameter<int>("timeout_ms", 500, descriptor) / 1000.0;
  config.coordinates_confirmed =
    declare_parameter<bool>("coordinates_confirmed", false, descriptor);
  config.position_scale = declare_parameter<double>("position_scale", 1.0, descriptor);
  config.y_sign = declare_parameter<double>("uwb_y_sign", 1.0, descriptor);
  config.yaw_rad = declare_parameter<double>("uwb_to_map_yaw_rad", 0.0, descriptor);
  const auto translation = declare_parameter<std::vector<double>>(
    "uwb_to_map_translation_m", {0.0, 0.0, 0.0}, descriptor);
  if (translation.size() != 3) {throw std::invalid_argument("UWB translation needs 3 values");}
  std::copy(translation.begin(), translation.end(), config.translation.begin());
  config.max_jump_m = declare_parameter<double>("max_jump_m", 0.0, descriptor);
  config.collect_static_noise = declare_parameter<bool>("collect_static_noise", false, descriptor);
  const auto sample_limit = declare_parameter<int>("static_noise_samples", 100, descriptor);
  if (sample_limit < 2 || sample_limit > 1000000) {
    throw std::invalid_argument("static_noise_samples must be in [2, 1000000]");
  }
  config.noise_sample_limit = static_cast<unsigned>(sample_limit);
  config.measured_stddev_xy_m = declare_parameter<double>("measured_stddev_xy_m", -1.0, descriptor);
  frame_id_ = declare_parameter<std::string>("frame_id", "map", descriptor);
  const auto topic = declare_parameter<std::string>(
    "input_topic", "/nlink_linktrack_anchorframe0", descriptor);
  const auto max_points = declare_parameter<int>("max_path_points", 1000, descriptor);
  if (frame_id_.empty() || max_points < 1 || max_points > 100000) {
    throw std::invalid_argument("Invalid frame_id or max_path_points");
  }
  max_path_points_ = static_cast<size_t>(max_points);
  processor_ = std::make_unique<UwbProcessor>(config);
  position_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position", 10);
  status_ = create_publisher<flapping_uav_msgs::msg::UwbStatus>("/uwb/status", 10);
  diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  const auto visual_qos = rclcpp::QoS(1).reliable().transient_local();
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("/uwb/path", visual_qos);
  markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/markers", visual_qos);
  path_.header.frame_id = frame_id_;
  // Upstream is reliable. A latest-only best-effort reader also accepts it and
  // prevents a long queue from turning delayed frames into fresh observations.
  input_ = create_subscription<nlink_parser2::msg::LinktrackAnchorframe0>(
    topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&UwbPositionNode::on_frame, this, std::placeholders::_1));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
    "/uwb/reset_observation", [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      processor_->reset_observation();
      last_position_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      path_.poses.clear(); path_.header.stamp = now(); path_publisher_->publish(path_);
      visualization_msgs::msg::Marker clear;
      clear.header.stamp = now(); clear.header.frame_id = frame_id_;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      visualization_msgs::msg::MarkerArray array; array.markers.push_back(clear);
      markers_->publish(array);
      response->success = true;
      response->message = "Observation baseline and static noise reset; counters retained";
      publish_status();
    });
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&UwbPositionNode::publish_status, this));
  RCLCPP_INFO(
    get_logger(), "Anchor_Frame0 %s -> Tag %d -> /uwb/position (%s)",
    topic.c_str(), config.tag_id, frame_id_.c_str());
  if (config.collect_static_noise) {
    RCLCPP_WARN(
      get_logger(), "Static noise collection enabled: keep the Tag stationary for %u samples",
      config.noise_sample_limit);
  }
}

void UwbPositionNode::on_frame(nlink_parser2::msg::LinktrackAnchorframe0::ConstSharedPtr message)
{
  UwbFrame frame;
  frame.role = message->role; frame.anchor_id = message->id;
  frame.local_time = message->local_time; frame.system_time = message->system_time;
  for (const auto & node : message->nodes) {
    frame.tags.push_back(
      UwbTag{node.role, node.id,
        {{node.pos_3d[0], node.pos_3d[1], node.pos_3d[2]}}});
  }
  const bool accepted = processor_->process(frame, monotonic_seconds());
  const auto state = processor_->snapshot(monotonic_seconds());
  if (accepted) {
    geometry_msgs::msg::PoseWithCovarianceStamped position;
    last_position_stamp_ = now();
    position.header.stamp = last_position_stamp_; position.header.frame_id = frame_id_;
    position.pose.pose.position.x = state.map_position[0];
    position.pose.pose.position.y = state.map_position[1];
    position.pose.pose.position.z = state.map_position[2];
    position.pose.pose.orientation.w = 1.0;  // Placeholder: UWB does not measure attitude.
    position.pose.covariance.fill(unknown());
    position.pose.covariance[0] = state.covariance_xy[0];
    position.pose.covariance[1] = state.covariance_xy[1];
    position.pose.covariance[6] = state.covariance_xy[2];
    position.pose.covariance[7] = state.covariance_xy[3];
    position_->publish(position);
    geometry_msgs::msg::PoseStamped projected;
    projected.header = position.header; projected.pose = position.pose.pose;
    projected.pose.position.z = 0.0;  // Explicit XY visualization projection, not altitude.
    path_.header = position.header; path_.poses.push_back(projected);
    if (path_.poses.size() > max_path_points_) {path_.poses.erase(path_.poses.begin());}
    path_publisher_->publish(path_);
    publish_visualization(state);
  }
  if (!last_logged_state_ || *last_logged_state_ != state.state) {publish_status();}
}

void UwbPositionNode::publish_status()
{
  const auto state = processor_->snapshot(monotonic_seconds());
  const auto & config = processor_->config();
  flapping_uav_msgs::msg::UwbStatus message;
  message.header.stamp = now(); message.header.frame_id = frame_id_;
  message.state = static_cast<uint8_t>(state.state); message.reason = state_name(state.state);
  message.tag_id = config.tag_id; message.configured_tag_role = config.tag_role;
  message.anchor_id = state.anchor_id; message.position_valid = state.position_valid;
  message.coordinates_confirmed = config.coordinates_confirmed;
  message.covariance_valid = state.covariance_valid;
  message.raw_position.x = state.raw_position[0]; message.raw_position.y = state.raw_position[1];
  message.raw_position.z = state.raw_position[2];
  message.frame_age_ms = state.frame_age_s * 1000;
  message.unique_frame_age_ms = state.unique_frame_age_s * 1000;
  message.position_age_ms = state.position_age_s * 1000;
  message.accepted_rate_hz = state.accepted_rate_hz;
  message.frames_received = state.frames_received;
  message.accepted_positions = state.accepted_positions;
  message.rejected_frames = state.rejected_frames;
  message.duplicate_frames = state.duplicate_frames;
  message.jump_rejections = state.jump_rejections;
  message.device_time_backwards = state.device_time_backwards;
  message.static_noise_samples = state.static_noise_samples;
  message.covariance_xy = state.covariance_xy;
  message.stddev_x_m = state.covariance_valid ? std::sqrt(state.covariance_xy[0]) : unknown();
  message.stddev_y_m = state.covariance_valid ? std::sqrt(state.covariance_xy[3]) : unknown();
  status_->publish(message);
  diagnostic_msgs::msg::DiagnosticStatus diagnostic;
  diagnostic.name = "flapping_uav/uwb"; diagnostic.hardware_id = "LinkTrack Tag " + std::to_string(
    config.tag_id);
  diagnostic.level = !state.position_valid ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    (state.covariance_valid ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::
    DiagnosticStatus::WARN);
  diagnostic.message = message.reason + (state.position_valid && !state.covariance_valid ?
    "; XY covariance not established" : "");
  const auto value = [&diagnostic](const std::string & name, const std::string & text) {
      diagnostic_msgs::msg::KeyValue item; item.key = name; item.value = text;
      diagnostic.values.push_back(item);
    };
  value("tag_id", std::to_string(config.tag_id));
  value("accepted_rate_hz", std::to_string(state.accepted_rate_hz));
  value("position_age_ms", std::to_string(message.position_age_ms));
  value("static_noise_samples", std::to_string(state.static_noise_samples));
  value("max_jump_check_enabled", config.max_jump_m > 0 ? "true" : "false");
  value("uwb_y_sign", std::to_string(config.y_sign));
  diagnostic_msgs::msg::DiagnosticArray array; array.header = message.header;
  array.status.push_back(diagnostic); diagnostics_->publish(array);
  if (!last_logged_state_ || *last_logged_state_ != state.state) {
    RCLCPP_INFO(get_logger(), "%s", message.reason.c_str()); last_logged_state_ = state.state;
  }
  publish_visualization(state);
}

void UwbPositionNode::publish_visualization(const UwbSnapshot & snapshot)
{
  if (!std::isfinite(snapshot.map_position[0]) || !std::isfinite(snapshot.map_position[1])) {
    return;
  }
  visualization_msgs::msg::Marker point;
  point.header.stamp = now(); point.header.frame_id = frame_id_;
  point.ns = "uwb_xy"; point.id = 0; point.type = visualization_msgs::msg::Marker::SPHERE;
  point.action = visualization_msgs::msg::Marker::ADD;
  point.pose.position.x = snapshot.map_position[0];
  point.pose.position.y = snapshot.map_position[1];
  point.pose.position.z = 0.0; point.pose.orientation.w = 1.0;
  point.scale.x = point.scale.y = point.scale.z = 0.12;
  point.color.a = 1.0; point.color.r = snapshot.position_valid ? 0.1F : 1.0F;
  point.color.g = snapshot.position_valid ? 0.9F : 0.1F;
  point.color.b = 0.1F;
  auto label = point;
  label.id = 1; label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.pose.position.z = 0.2; label.scale.z = 0.12;
  label.text = "Tag " + std::to_string(processor_->config().tag_id) + ": " + state_name(
    snapshot.state);
  visualization_msgs::msg::MarkerArray array;
  array.markers = {point, label}; markers_->publish(array);
}
}  // namespace flapping_uav_gcs
