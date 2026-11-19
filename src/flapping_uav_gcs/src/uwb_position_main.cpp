#include <iostream>
#include "flapping_uav_gcs/uwb_position_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<flapping_uav_gcs::UwbPositionNode>());
  } catch (const std::exception & error) {
    std::cerr << "uwb_position_node: " << error.what() << '\n'; result = 1;
  }
  rclcpp::shutdown();
  return result;
}
