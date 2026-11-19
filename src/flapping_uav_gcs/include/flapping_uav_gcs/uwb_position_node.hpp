#pragma once

#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <flapping_uav_msgs/msg/uwb_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlink_parser2/msg/linktrack_anchorframe0.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "flapping_uav_gcs/uwb_processor.hpp"

namespace flapping_uav_gcs
{
class UwbPositionNode : public rclcpp::Node
{
public:
  explicit UwbPositionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_frame(nlink_parser2::msg::LinktrackAnchorframe0::ConstSharedPtr message);
  void publish_status();
  void publish_visualization(const UwbSnapshot & snapshot);
  std::unique_ptr<UwbProcessor> processor_;
  std::string frame_id_;
  size_t max_path_points_{1000};
  rclcpp::Time last_position_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<UwbState> last_logged_state_;
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nlink_parser2::msg::LinktrackAnchorframe0>::SharedPtr input_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_;
  rclcpp::Publisher<flapping_uav_msgs::msg::UwbStatus>::SharedPtr status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
double monotonic_seconds();
}  // namespace flapping_uav_gcs
