#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "flapping_uav_gcs/uwb_position_node.hpp"

using namespace std::chrono_literals;  // NOLINT
using flapping_uav_msgs::msg::UwbStatus;

TEST(UwbRos, ActualDdsFiltersTagDetectsFailuresRecoversAndBoundsThePathWithPausedRosClock)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("tag_id", 0),
        rclcpp::Parameter("coordinates_confirmed", true), rclcpp::Parameter("timeout_ms", 120),
        rclcpp::Parameter("max_path_points", 3), rclcpp::Parameter("use_sim_time", true)});
    auto pipeline = std::make_shared<flapping_uav_gcs::UwbPositionNode>(options);
    auto probe = std::make_shared<rclcpp::Node>("uwb_stage1_test_probe");
    auto input = probe->create_publisher<nlink_parser2::msg::LinktrackAnchorframe0>(
      "/nlink_linktrack_anchorframe0", 10);
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> positions;
    UwbStatus last_status;
    nav_msgs::msg::Path last_path;
    auto pos_sub = probe->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/position", 10,
      [&positions](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        positions.push_back(*m);
      });
    auto status_sub = probe->create_subscription<UwbStatus>(
      "/uwb/status", 10,
      [&last_status](UwbStatus::ConstSharedPtr m) {last_status = *m;});
    auto path_sub = probe->create_subscription<nav_msgs::msg::Path>(
      "/uwb/path", 10,
      [&last_path](nav_msgs::msg::Path::ConstSharedPtr m) {last_path = *m;});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pipeline); executor.add_node(probe);
    const auto until = [&executor](const std::function<bool()> & predicate, double seconds) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (!predicate() && std::chrono::steady_clock::now() < end) {
          executor.spin_some(); std::this_thread::sleep_for(2ms);
        }
        return predicate();
      };
    ASSERT_TRUE(
      until(
        [&]() {
          return input->get_subscription_count() > 0 &&
          pipeline->count_subscribers("/uwb/position") > 0;
        }, 3));
    nlink_parser2::msg::LinktrackAnchorframe0 frame;
    frame.id = 1; frame.local_time = frame.system_time = 1;
    nlink_parser2::msg::LinktrackTag other; other.id = 99; other.pos_3d = {{99, 99, 99}};
    nlink_parser2::msg::LinktrackTag target; target.id = 0; target.pos_3d = {{1, 2, .8F}};
    frame.nodes = {other, target}; input->publish(frame);
    ASSERT_TRUE(until([&]() {return positions.size() == 1 && last_status.position_valid;}, 1));
    EXPECT_DOUBLE_EQ(positions.back().pose.pose.position.x, 1);
    EXPECT_DOUBLE_EQ(positions.back().pose.pose.position.y, 2);
    EXPECT_EQ(positions.back().header.frame_id, "map");
    EXPECT_EQ(positions.back().header.stamp.sec, 0);  // Paused ROS /clock.
    EXPECT_TRUE(std::isnan(positions.back().pose.covariance[0]));
    frame.local_time = frame.system_time = 2; frame.nodes = {other}; input->publish(frame);
    ASSERT_TRUE(until([&]() {return last_status.state == UwbStatus::TARGET_MISSING;}, 1));
    EXPECT_EQ(positions.size(), 1U);
    frame.local_time = frame.system_time = 3;
    target.pos_3d[0] = std::numeric_limits<float>::quiet_NaN();
    frame.nodes = {other, target}; input->publish(frame);
    ASSERT_TRUE(until([&]() {return last_status.state == UwbStatus::NONFINITE_POSITION;}, 1));
    EXPECT_EQ(positions.size(), 1U);
    frame.local_time = frame.system_time = 4; target.pos_3d[0] = 1.1F;
    frame.nodes = {other, target}; input->publish(frame);
    ASSERT_TRUE(until([&]() {return positions.size() == 2 && last_status.position_valid;}, 1));
    ASSERT_TRUE(until([&]() {return last_status.state == UwbStatus::STREAM_TIMEOUT;}, 1));
    EXPECT_EQ(positions.size(), 2U); EXPECT_FALSE(last_status.position_valid);
    for (int i = 5; i < 10; ++i) {
      frame.local_time = frame.system_time = i; input->publish(frame);
      ASSERT_TRUE(until([&]() {return positions.size() == static_cast<size_t>(i - 2);}, 1));
    }
    ASSERT_TRUE(
      until(
        [&]() {
          return last_path.poses.size() == 3 && last_status.position_valid;
        }, 1));
    EXPECT_EQ(last_path.header.frame_id, "map");
    EXPECT_DOUBLE_EQ(last_path.poses.back().pose.position.z, 0);  // Visualization projection only.
    EXPECT_NEAR(positions.back().pose.pose.position.z, .8, 1e-6);  // Measurement Z preserved.
    executor.remove_node(probe); executor.remove_node(pipeline);
  }
  rclcpp::shutdown();
}
