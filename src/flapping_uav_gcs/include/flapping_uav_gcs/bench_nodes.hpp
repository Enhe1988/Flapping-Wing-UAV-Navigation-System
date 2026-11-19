#ifndef FLAPPING_UAV_GCS__BENCH_NODES_HPP_
#define FLAPPING_UAV_GCS__BENCH_NODES_HPP_
#include "rclcpp/rclcpp.hpp"
namespace flapping_uav_gcs
{
rclcpp::Node::SharedPtr make_fusion_odometry_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
rclcpp::Node::SharedPtr make_manual_control_node(const rclcpp::NodeOptions & options);
rclcpp::Node::SharedPtr make_mavlink_bridge_node(const rclcpp::NodeOptions & options);
rclcpp::Node::SharedPtr make_velocity_transform_node(const rclcpp::NodeOptions & options);
rclcpp::Node::SharedPtr make_position_controller_node(const rclcpp::NodeOptions & options);
rclcpp::Node::SharedPtr make_scene_visualizer_node(const rclcpp::NodeOptions & options);
}  // namespace flapping_uav_gcs
#endif  // FLAPPING_UAV_GCS__BENCH_NODES_HPP_
