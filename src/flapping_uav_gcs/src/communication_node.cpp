#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "joy/joy.hpp"
#include "flapping_uav_gcs/bench_nodes.hpp"

// One process owns joystick acquisition, channel mapping and UDP communication.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto host = std::make_shared<rclcpp::Node>("communication_node", options);
    auto child_options = [&](const std::string & prefix) {
        std::vector<rclcpp::Parameter> parameters;
        for (const auto & name : host->list_parameters({prefix}, 10).names) {
          parameters.emplace_back(name.substr(prefix.size()+1), host->get_parameter(name).get_parameter_value());
        }
        rclcpp::NodeOptions child;
        child.use_global_arguments(false).parameter_overrides(parameters);
        return child;
      };
    auto manual = flapping_uav_gcs::make_manual_control_node(child_options("manual"));
    auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(child_options("bridge"));
    std::shared_ptr<joy::Joy> joystick;
    bool use_joy = true;
    host->get_parameter_or("use_joy", use_joy, true);
    if (use_joy) {joystick = std::make_shared<joy::Joy>(child_options("joy"));}
    rclcpp::executors::SingleThreadedExecutor executor;
    for (auto node : {host, manual, bridge}) {executor.add_node(node);}
    if (joystick) {executor.add_node(joystick);}
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("communication_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
