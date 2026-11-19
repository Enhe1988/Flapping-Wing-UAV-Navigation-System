#include <fcntl.h>
#include <linux/joystick.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "flapping_uav_gcs/rc_mapping.hpp"
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_msgs/msg/rc_override.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace flapping_uav_gcs
{
class ManualControlNode : public rclcpp::Node
{
public:
  explicit ManualControlNode(const rclcpp::NodeOptions & options)
  : Node("manual_control_node", options)
  {
    const auto indices = startup_parameter<std::vector<int64_t>>(
      "axis_indices", {0, 1, 2, 3, 4, 5, 6, 7});
    const auto signs = startup_parameter<std::vector<double>>(
      "axis_signs", {-1., -1., -1., -1., -1., -1., -1., -1.});
    const auto centers = startup_parameter<std::vector<double>>(
      "axis_centers", {0., 0., 0., 0., 0., 0., 0., 0.});
    const auto fixed = startup_parameter<std::vector<int64_t>>(
      "fixed_channels", std::vector<int64_t>(mapping_.fixed.begin(), mapping_.fixed.end()));
    if (indices.size() != 8 || signs.size() != 8 || centers.size() != 8 || fixed.size() != 18) {
      throw std::invalid_argument("Expected 8 axis settings and 18 fixed_channels");
    }
    for (size_t i = 0; i < 8; ++i) {
      if (indices[i] < 0 || indices[i] > 63) {
        throw std::invalid_argument("Invalid axis index");
      }
      mapping_.indices[i] = static_cast<int>(indices[i]);
      mapping_.signs[i] = signs[i]; mapping_.centers[i] = centers[i];
    }
    for (size_t i = 0; i < 18; ++i) {
      if (fixed[i] < 0 || fixed[i] > 65535) {
        throw std::invalid_argument("Invalid fixed channel value");
      }
      mapping_.fixed[i] = static_cast<uint16_t>(fixed[i]);
    }
    mapping_.deadzone = startup_parameter("stick_deadzone", 0.02);
    mapping_.validate();
    timeout_ms_ = startup_parameter("input_timeout_ms", 200);
    const auto rate = startup_parameter("publish_rate_hz", 50.0);
    if (timeout_ms_ <= 0 || !std::isfinite(rate) || rate < 1 || rate > 200) {
      throw std::invalid_argument("Invalid RC timeout or publication rate");
    }
    verify_device_ = startup_parameter("verify_device", true);
    device_path_ = startup_parameter(
      "device_path", std::string(
        "/dev/input/by-id/usb-EdgeTX_Radiomaster_Pocket_Joystick_00000000001B-joystick"));
    expected_name_ = startup_parameter(
      "device_name", std::string("EdgeTX Radiomaster Pocket Joystick"));
    output_ = create_publisher<flapping_uav_msgs::msg::RcOverride>("/rc/override", 10);
    input_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr message) {
        ++input_frames_;
        last_input_ = Clock::now(); seen_input_ = true;
        mapping_valid_ = mapping_.convert(message->axes, channels_);
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1 / rate)), [this]() {publish();});
    RCLCPP_INFO(
      get_logger(), "RC preview on /rc/override; device verification=%s",
      verify_device_ ? "true" : "false (simulation only)");
  }

  ~ManualControlNode() override {if (device_fd_ >= 0) {::close(device_fd_);}}

private:
  using Clock = std::chrono::steady_clock;
  template<typename T>
  T startup_parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  bool device_available()
  {
    if (!verify_device_) {return true;}
    if (device_lost_) {return false;}
    if (device_fd_ < 0) {
      device_fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (device_fd_ < 0) {return false;}
      char name[128]{};
      if (::ioctl(device_fd_, JSIOCGNAME(sizeof(name)), name) < 0 ||
        std::string(name) != expected_name_)
      {
        ::close(device_fd_); device_fd_ = -1;
        return false;
      }
      // Do not promote a cached Joy frame received before opening this device.
      seen_input_ = false;
    }
    pollfd event{device_fd_, POLLIN, 0};
    if (::poll(&event, 1, 0) < 0 || (event.revents & (POLLHUP | POLLERR | POLLNVAL))) {
      device_lost_ = true;
    } else if (event.revents & POLLIN) {
      std::array<js_event, 64> buffer;
      // Independent read-only joystick fd; never grabs or consumes SDL's fd.
      if (::read(device_fd_, buffer.data(), sizeof(buffer)) == 0) {device_lost_ = true;}
    }
    if (device_lost_) {
      ::close(device_fd_); device_fd_ = -1;
      RCLCPP_ERROR(get_logger(), "Pocket disconnected: restart joy/manual nodes before resuming");
      return false;
    }
    return true;
  }

  void publish()
  {
    const bool connected = device_available();
    const double age = seen_input_ ?
      std::chrono::duration<double, std::milli>(Clock::now() - last_input_).count() :
      std::numeric_limits<double>::quiet_NaN();
    flapping_uav_msgs::msg::RcOverride message;
    message.header.stamp = now();
    message.device_connected = connected;
    message.input_age_ms = age;
    message.input_frames = input_frames_;
    message.valid = connected && seen_input_ && mapping_valid_ && age <= timeout_ms_;
    if (!connected) {
      message.reason = "Pocket missing or disconnected; verify device path/restart";
    } else if (!seen_input_) {message.reason = "Waiting for /joy";} else if (!mapping_valid_) {
      message.reason = "Invalid/nonfinite/incomplete Joy axes";
    } else if (age > timeout_ms_) {message.reason = "Joy input timeout";} else {
      message.reason = "valid bench mapping";
    }
    message.channels = message.valid ? channels_ : mapping_.fixed;
    output_->publish(message);
  }

  RcMapping mapping_;
  std::array<uint16_t, 18> channels_{};
  int timeout_ms_{200}, device_fd_{-1};
  bool verify_device_{true}, device_lost_{false}, seen_input_{false}, mapping_valid_{false};
  std::string device_path_, expected_name_;
  Clock::time_point last_input_{};
  uint64_t input_frames_{0};
  rclcpp::Publisher<flapping_uav_msgs::msg::RcOverride>::SharedPtr output_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
rclcpp::Node::SharedPtr make_manual_control_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ManualControlNode>(options);
}
}  // namespace flapping_uav_gcs

#ifndef FLAPPING_UAV_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(flapping_uav_gcs::make_manual_control_node(rclcpp::NodeOptions()));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("manual_control_node"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
#endif
