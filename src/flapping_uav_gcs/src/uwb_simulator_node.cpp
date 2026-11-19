#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

#include <nlink_parser2/msg/linktrack_anchorframe0.hpp>
#include <nlink_parser2/msg/linktrack_tag.hpp>
#include <rclcpp/rclcpp.hpp>

class UwbSimulator : public rclcpp::Node
{
public:
  UwbSimulator()
  : Node("uwb_simulator_node")
  {
    rate_ = declare_parameter<double>("rate_hz", 20.0);
    tag_id_ = declare_parameter<int>("tag_id", 0);
    scenario_ = declare_parameter<std::string>("scenario", "normal");
    const auto topic = declare_parameter<std::string>(
      "output_topic",
      "/nlink_linktrack_anchorframe0");
    const std::vector<std::string> scenarios = {"normal", "missing_tag", "nonfinite", "jump",
      "duplicate", "dropout", "wrap_restart"};
    if (!std::isfinite(rate_) || rate_ <= 0 || rate_ > 500 || tag_id_ < 0 || tag_id_ > 255 ||
      std::find(scenarios.begin(), scenarios.end(), scenario_) == scenarios.end())
    {
      throw std::invalid_argument("Invalid simulator rate, Tag or scenario");
    }
    publisher_ = create_publisher<nlink_parser2::msg::LinktrackAnchorframe0>(topic, 10);
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate_)),
      [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "SIMULATION only: Anchor_Frame0, Tag %d, %.1f Hz, %s",
      tag_id_, rate_, scenario_.c_str());
  }

private:
  void tick()
  {
    const auto index = count_++;
    if (scenario_ == "dropout" && static_cast<int>(index / rate_) % 4 == 2) {return;}
    nlink_parser2::msg::LinktrackAnchorframe0 frame;
    frame.role = 1; frame.id = 1; frame.voltage = 5.0F;  // Synthetic device identities.
    uint32_t stamp = static_cast<uint32_t>(index * 1000.0 / rate_);
    if (scenario_ == "duplicate") {stamp = 1;}
    if (scenario_ == "wrap_restart") {
      stamp = index < 10 ? static_cast<uint32_t>(0xffffff00ULL + index * 50) :
        static_cast<uint32_t>((index - 10) * 50);
    }
    frame.local_time = frame.system_time = stamp;
    nlink_parser2::msg::LinktrackTag other;
    other.id = static_cast<uint8_t>((tag_id_ + 1) % 256);
    other.pos_3d = {{99.0F, 99.0F, 99.0F}};
    frame.nodes.push_back(other);  // Target is deliberately not nodes[0].
    if (scenario_ != "missing_tag") {
      nlink_parser2::msg::LinktrackTag tag;
      tag.role = 0; tag.id = static_cast<uint8_t>(tag_id_);
      tag.pos_3d = {{static_cast<float>(1.0 + .01 * std::sin(index * .7)),
        static_cast<float>(2.0 + .015 * std::cos(index * .4)), .8F}};
      if (scenario_ == "nonfinite") {tag.pos_3d[0] = std::numeric_limits<float>::quiet_NaN();}
      if (scenario_ == "jump" && index > rate_ * 2) {tag.pos_3d[0] += 10.0F;}
      frame.nodes.push_back(tag);
    }
    publisher_->publish(frame);
  }
  double rate_{20}; int tag_id_{0}; std::string scenario_; uint64_t count_{0};
  rclcpp::Publisher<nlink_parser2::msg::LinktrackAnchorframe0>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<UwbSimulator>());
  } catch (const std::exception & error) {
    std::cerr << "uwb_simulator_node: " << error.what() << '\n'; result = 1;
  }
  rclcpp::shutdown();
  return result;
}
