#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "mavlink_codec.hpp"
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_gcs/sample_order.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "flapping_uav_msgs/msg/mavlink_frame.hpp"
#include "flapping_uav_msgs/msg/rc_override.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flapping_uav_gcs
{
struct UdpFd
{
  int value{-1};
  ~UdpFd() {if (value >= 0) {::close(value);}}
};

class MavlinkBridgeNode : public rclcpp::Node
{
public:
  explicit MavlinkBridgeNode(const rclcpp::NodeOptions & options)
  : Node("mavlink_bridge_node", options)
  {
    tx_enabled_ = startup_parameter("tx_enabled", false);
    send_heartbeat_ = startup_parameter("send_heartbeat", true);
    remote_ip_ = startup_parameter("remote_ip", std::string(""));
    const auto bind_ip = startup_parameter("bind_ip", std::string("0.0.0.0"));
    rx_port_ = startup_parameter("rx_port", 9000);
    tx_port_ = startup_parameter("tx_port", 9111);
    system_id_ = startup_parameter("system_id", 255);
    component_id_ = startup_parameter("component_id", 190);
    target_system_ = startup_parameter("target_system", 1);
    target_component_ = startup_parameter("target_component", 1);
    rc_timeout_ms_ = startup_parameter("rc_timeout_ms", 250);
    const auto rc_rate = startup_parameter("rc_rate_hz", 40.0);
    print_rx_ = startup_parameter("print_rx", true);
    loss_action_ = startup_parameter("rc_loss_action", std::string("stop"));
    send_velocity_ = startup_parameter("send_velocity", false);
    velocity_y_sign_ = startup_parameter("velocity_y_sign", 1);
    if (velocity_y_sign_ != 1 && velocity_y_sign_ != -1) {
      throw std::invalid_argument("velocity_y_sign must be +1 or -1");
    }
    velocity_xy_only_ = startup_parameter("velocity_xy_only", false);
    velocity_frame_id_ = startup_parameter("velocity_frame_id", std::string("base_link_frd"));
    const bool heading_confirmed = startup_parameter("heading_xy_profile_confirmed", false);
    if (velocity_frame_id_ != "base_link_frd" &&
      (velocity_frame_id_ != "base_link_heading_frd" || !velocity_xy_only_ || !heading_confirmed))
    {throw std::invalid_argument("Heading XY requires explicit agreed profile and ignored vz");}
    motion_name_ = startup_parameter("motion_name", std::string(""));
    const auto velocity_rate = startup_parameter("velocity_rate_hz", 30.0);
    velocity_timeout_ms_ = startup_parameter("velocity_timeout_ms", 200);
    velocity_attitude_timeout_ms_ = startup_parameter(
      "velocity_attitude_timeout_ms", velocity_timeout_ms_);
    max_body_speed_ = startup_parameter("max_body_speed_mps", 1.0);
    attitude_message_id_ = startup_parameter("attitude_message_id", 31);
    attitude_system_ = startup_parameter("attitude_system_id", target_system_);
    attitude_component_ = startup_parameter("attitude_component_id", target_component_);
    if (rx_port_ < 1 || rx_port_ > 65535 || tx_port_ < 1 || tx_port_ > 65535 ||
      system_id_ < 1 || system_id_ > 255 || component_id_ < 0 || component_id_ > 255 ||
      target_system_ < 1 || target_system_ > 255 || target_component_ < 0 ||
      target_component_ > 255 ||
      rc_timeout_ms_ <= 0 || !std::isfinite(rc_rate) || rc_rate < 1 || rc_rate > 100 ||
      (loss_action_ != "stop" && loss_action_ != "release"))
    {throw std::invalid_argument("Invalid MAVLink endpoint, ID, rate, timeout or loss action");}
    if (!std::isfinite(velocity_rate) || velocity_rate < 1 || velocity_rate > 100 ||
      velocity_timeout_ms_ <= 0 || velocity_attitude_timeout_ms_ <= 0 || !std::isfinite(max_body_speed_) || max_body_speed_ <= 0 ||
      (attitude_message_id_ != 30 && attitude_message_id_ != 31 && attitude_message_id_ != 331) ||
      attitude_system_ < 1 || attitude_system_ > 255 ||
      attitude_component_ < 0 || attitude_component_ > 255)
    {throw std::invalid_argument("Invalid body velocity/attitude configuration");}
    sockaddr_in local{}; local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(rx_port_));
    if (::inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr) != 1) {
      throw std::invalid_argument("bind_ip must be a numeric IPv4 address");
    }
    remote_.sin_family = AF_INET; remote_.sin_port = htons(static_cast<uint16_t>(tx_port_));
    if (tx_enabled_ && ::inet_pton(AF_INET, remote_ip_.c_str(), &remote_.sin_addr) != 1) {
      throw std::invalid_argument("Set remote_ip explicitly before enabling TX");
    }
    socket_.value = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_.value < 0) {throw std::runtime_error(std::strerror(errno));}
    if (::bind(socket_.value, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0) {
      throw std::runtime_error(
              "Cannot bind UDP " + bind_ip + ":" +
              std::to_string(rx_port_) + ": " + std::strerror(errno));
    }
    rx_output_ = create_publisher<flapping_uav_msgs::msg::MavlinkFrame>("/fc/mavlink_rx", 100);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    attitude_output_ = create_publisher<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude", 10);
    motion_output_ = create_publisher<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion", 10);
    sent_velocity_output_ = create_publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
      "/fc/velocity_setpoint_sent", 10);
    velocity_input_ = create_subscription<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
      "/fc/velocity_setpoint_body", rclcpp::QoS(1),
      [this](flapping_uav_msgs::msg::BodyVelocitySetpoint::ConstSharedPtr message) {
        last_velocity_ = *message; last_velocity_time_ = Clock::now(); seen_velocity_ = true;
      });
    rc_input_ = create_subscription<flapping_uav_msgs::msg::RcOverride>(
      "/rc/override", rclcpp::QoS(1),
      [this](flapping_uav_msgs::msg::RcOverride::ConstSharedPtr message) {
        last_rc_ = *message; last_rc_time_ = Clock::now(); seen_rc_ = true;
      });
    rx_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {receive();});
    tx_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1 / rc_rate)), [this]() {send_rc();});
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {
        if (tx_enabled_ && send_heartbeat_) {
          mavlink_message_t message;
          mavlink_msg_heartbeat_pack(
            system_id_, component_id_, &message,
            MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
          transmit(message);
        }
        publish_status();
      });
    velocity_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1 / velocity_rate)), [this]() {send_body_velocity();});
    RCLCPP_INFO(
      get_logger(), "RX UDP %s:%d; TX=%s, destination=%s:%d; loss=%s",
      bind_ip.c_str(), rx_port_, tx_enabled_ ? "enabled" : "disabled",
      remote_ip_.c_str(), tx_port_, loss_action_.c_str());
    RCLCPP_INFO(
      get_logger(), "Velocity ID84: %s; frame8, input=%s, XY-only=%s; wire vy sign=%d",
      send_velocity_ ? "enabled when TX enabled" : "disabled", velocity_frame_id_.c_str(),
      velocity_xy_only_ ? "true" : "false", velocity_y_sign_);
  }

private:
  using Clock = std::chrono::steady_clock;
  template<typename T>
  T startup_parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  struct Stream
  {
    mavlink_message_t buffer{};
    mavlink_status_t status{};
    Clock::time_point last{};
  };

  bool rc_fresh() const
  {
    return seen_rc_ && last_rc_.valid && last_rc_.device_connected &&
           std::isfinite(last_rc_.input_age_ms) && last_rc_.input_age_ms >= 0 &&
           last_rc_.input_age_ms +
           std::chrono::duration<double, std::milli>(Clock::now() - last_rc_time_).count() <=
           rc_timeout_ms_;
  }

  bool transmit(const mavlink_message_t & message)
  {
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes;
    const auto length = mavlink_msg_to_send_buffer(bytes.data(), &message);
    const auto sent = ::sendto(
      socket_.value, bytes.data(), length, 0,
      reinterpret_cast<const sockaddr *>(&remote_), sizeof(remote_));
    if (sent != length) {++tx_errors_; return false;}
    ++tx_frames_; return true;
  }

  void send_rc()
  {
    if (!tx_enabled_) {return;}
    const bool fresh = rc_fresh();
    if (!fresh && loss_action_ == "stop") {return;}
    auto channels = last_rc_.channels;
    if (!fresh) {
      for (size_t i = 0; i < channels.size(); ++i) {channels[i] = i < 8 ? 0 : 65534;}
    } else {
      for (auto value : channels) {
        if (!((value >= 1000 && value <= 2000) || value == 65535)) {return;}
      }
    }
    mavlink_message_t message;
    mavlink_msg_rc_channels_override_pack(
      system_id_, component_id_, &message,
      target_system_, target_component_, channels[0], channels[1], channels[2], channels[3],
      channels[4], channels[5], channels[6], channels[7], channels[8], channels[9], channels[10],
      channels[11], channels[12], channels[13], channels[14], channels[15], channels[16],
      channels[17]);
    transmit(message); ++rc_frames_;
  }

  bool velocity_fresh() const
  {
    const auto & v = last_velocity_.velocity;
    const double elapsed = std::chrono::duration<double, std::milli>(
      Clock::now() - last_velocity_time_).count();
    return seen_velocity_ && last_velocity_.valid &&
           last_velocity_.header.frame_id == velocity_frame_id_ &&
           std::isfinite(last_velocity_.command_age_ms) && last_velocity_.command_age_ms >= 0 &&
           std::isfinite(last_velocity_.attitude_age_ms) && last_velocity_.attitude_age_ms >= 0 &&
           elapsed + last_velocity_.command_age_ms <= velocity_timeout_ms_ &&
           elapsed + last_velocity_.attitude_age_ms <= velocity_attitude_timeout_ms_ &&
           finite_vector({{v.x, v.y, v.z}}) && std::hypot(v.x, v.y, v.z) <= max_body_speed_ &&
           (velocity_frame_id_ != "base_link_heading_frd" || v.z == 0);
  }

  void send_body_velocity()
  {
    if (!tx_enabled_ || !send_velocity_) {return;}
    const bool valid = velocity_fresh();
    const auto & v = last_velocity_.velocity;
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      Clock::now() - start_time_).count();
    static_assert(MAV_FRAME_BODY_NED == 8, "ID84 body-velocity interface changed");
    constexpr uint16_t base_mask = POSITION_TARGET_TYPEMASK_X_IGNORE |
      POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
      POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |
      POSITION_TARGET_TYPEMASK_AZ_IGNORE | POSITION_TARGET_TYPEMASK_YAW_IGNORE |
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    static_assert(base_mask == 3527, "Velocity-only mask changed");
    const uint16_t mask = base_mask |
      (velocity_xy_only_ ? POSITION_TARGET_TYPEMASK_VZ_IGNORE : 0);
    mavlink_message_t message;
    // Generic frame8 uses true FRD. The explicitly confirmed planar application
    // profile instead interprets vx/vy as horizontal forward/right; ignore vz.
    // Both ends must select the same profile; the wire alone cannot distinguish it.
    mavlink_msg_set_position_target_local_ned_pack(
      system_id_, component_id_, &message,
      static_cast<uint32_t>(elapsed), target_system_, target_component_, MAV_FRAME_BODY_NED,
      mask, 0, 0, 0, valid ? v.x : 0, valid ? velocity_y_sign_ * v.y : 0,
      valid && !velocity_xy_only_ ? v.z : 0, 0, 0, 0, 0, 0);
    if (transmit(message)) {
      // Decode the actual packed float fields for the monitor topic. This is
      // the FC application command convention, not the internal FRD frame.
      mavlink_set_position_target_local_ned_t wire;
      mavlink_msg_set_position_target_local_ned_decode(&message, &wire);
      flapping_uav_msgs::msg::BodyVelocitySetpoint sent;
      sent.header.stamp = now(); sent.header.frame_id = "fc_velocity_command";
      sent.valid = valid;
      sent.reason = valid ? "UDP ID84 sent; vy sign=" + std::to_string(velocity_y_sign_) :
        "UDP ID84 zero sent: velocity input missing/invalid/stale";
      const double age_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - last_velocity_time_).count();
      sent.command_age_ms = seen_velocity_ ? last_velocity_.command_age_ms + age_ms :
        std::numeric_limits<double>::quiet_NaN();
      sent.attitude_age_ms = seen_velocity_ ? last_velocity_.attitude_age_ms + age_ms :
        std::numeric_limits<double>::quiet_NaN();
      sent.velocity.x = wire.vx; sent.velocity.y = wire.vy; sent.velocity.z = wire.vz;
      sent_velocity_output_->publish(sent);
    }
    ++velocity_frames_;
  }

  bool accept_sample(SampleOrderFilter & filter, uint64_t device_us, uint8_t reset_counter,
    const char * label, uint64_t & drops, uint64_t & resets)
  {
    const auto previous = filter.last_time_us();
    const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
    const auto result = filter.observe(device_us, reset_counter, host_ns);
    if (result == SampleOrder::duplicate || result == SampleOrder::old_packet) {
      ++drops;
      if (result == SampleOrder::old_packet) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "%s old/reordered sample dropped: device_us=%llu, latest_us=%llu",
          label, static_cast<unsigned long long>(device_us),
          static_cast<unsigned long long>(previous));
      }
      return false;
    }
    if (result == SampleOrder::clock_reset) {
      ++resets;
      RCLCPP_WARN(get_logger(), "%s clock/reference reset confirmed: %llu -> %llu us",
        label, static_cast<unsigned long long>(previous),
        static_cast<unsigned long long>(device_us));
    }
    return true;
  }

  void publish_motion(const mavlink_message_t & message)
  {
    if (message.msgid != MAVLINK_MSG_ID_DEBUG_VECT ||
      message.sysid != static_cast<uint32_t>(target_system_) ||
      message.compid != target_component_) {return;}
    mavlink_debug_vect_t value;
    mavlink_msg_debug_vect_decode(&message, &value);
    const std::string name(value.name, strnlen(value.name, sizeof(value.name)));
    if (!motion_name_.empty() && name != motion_name_) {return;}
    flapping_uav_msgs::msg::BodyMotionState output;
    output.header.stamp = now(); output.header.frame_id = "body_xy";
    output.system_id = message.sysid; output.component_id = message.compid;
    output.sample_time_us = value.time_usec; output.name = name;
    output.forward_velocity_mps = value.x; output.right_velocity_mps = value.y;
    output.height_up_m = value.z;
    output.velocity_valid = std::isfinite(value.x) && std::isfinite(value.y);
    output.height_valid = std::isfinite(value.z);
    output.reason = output.velocity_valid ? "ID250 XY velocity; z is height, not vz" :
      "ID250 has nonfinite XY velocity";
    if (accept_sample(motion_order_, output.sample_time_us, 0, "ID250",
      motion_order_drops_, motion_clock_resets_)) {motion_output_->publish(output);}
  }

  void publish_attitude(const mavlink_message_t & message)
  {
    if (message.msgid != static_cast<uint32_t>(attitude_message_id_) ||
      message.sysid != static_cast<uint32_t>(attitude_system_) ||
      message.compid != attitude_component_) {return;}
    flapping_uav_msgs::msg::AttitudeState output;
    output.header.stamp = now(); output.header.frame_id = "local_ned";
    output.body_frame_id = "base_link_frd";
    output.system_id = message.sysid; output.component_id = message.compid;
    output.message_id = message.msgid;
    QuaternionWxyz q;
    bool metadata_valid = true;
    if (message.msgid == 31) {
      mavlink_attitude_quaternion_t value;
      mavlink_msg_attitude_quaternion_decode(&message, &value);
      q = {{value.q1, value.q2, value.q3, value.q4}};
      output.sample_time_us = static_cast<uint64_t>(value.time_boot_ms) * 1000;
    } else if (message.msgid == 30) {
      mavlink_attitude_t value; mavlink_msg_attitude_decode(&message, &value);
      const double cr = std::cos(value.roll / 2), sr = std::sin(value.roll / 2);
      const double cp = std::cos(value.pitch / 2), sp = std::sin(value.pitch / 2);
      const double cy = std::cos(value.yaw / 2), sy = std::sin(value.yaw / 2);
      q = {{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy}};
      output.sample_time_us = static_cast<uint64_t>(value.time_boot_ms) * 1000;
    } else {
      mavlink_odometry_t value; mavlink_msg_odometry_decode(&message, &value);
      q = {{value.q[0], value.q[1], value.q[2], value.q[3]}};
      output.sample_time_us = value.time_usec; output.reset_counter = value.reset_counter;
      metadata_valid = value.frame_id == MAV_FRAME_LOCAL_NED &&
        value.child_frame_id == MAV_FRAME_BODY_FRD && value.quality != -1;
    }
    output.valid = normalize_attitude(q) && metadata_valid;
    output.orientation.w = q[0]; output.orientation.x = q[1];
    output.orientation.y = q[2]; output.orientation.z = q[3];
    output.reason =
      output.valid ? "Numeric attitude valid; physical convention requires verification" :
      "Nonfinite/nonunit attitude, wrong odometry frames or failed estimator";
    if (accept_sample(attitude_order_, output.sample_time_us, output.reset_counter, "Attitude",
      attitude_order_drops_, attitude_clock_resets_)) {attitude_output_->publish(output);}
  }

  void receive()
  {
    std::array<uint8_t, 65536> bytes;
    for (unsigned packet = 0; packet < 64; ++packet) {
      sockaddr_in peer{}; socklen_t size = sizeof(peer);
      const auto length = ::recvfrom(
        socket_.value, bytes.data(), bytes.size(), 0,
        reinterpret_cast<sockaddr *>(&peer), &size);
      if (length < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {++rx_errors_;}
        break;
      }
      ++rx_datagrams_;
      char ip[INET_ADDRSTRLEN]{};
      ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
      const std::string source = std::string(ip) + ":" + std::to_string(ntohs(peer.sin_port));
      const auto now = Clock::now();
      for (auto entry = streams_.begin(); entry != streams_.end(); ) {
        if (now - entry->second.last > std::chrono::seconds(60)) {
          entry = streams_.erase(entry);
        } else {++entry;}
      }
      if (streams_.find(source) == streams_.end() && streams_.size() >= 32) {
        ++source_drops_; continue;
      }
      auto & stream = streams_[source];
      if (now - stream.last > std::chrono::seconds(1)) {
        stream.buffer = {}; stream.status = {};
      }
      stream.last = now;
      for (ssize_t i = 0; i < length; ++i) {
        mavlink_message_t message{}; mavlink_status_t status{};
        const auto framing = mavlink_frame_char_buffer(
          &stream.buffer, &stream.status, bytes[i], &message, &status);
        if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
          ++crc_errors_; continue;
        }
        if (framing != MAVLINK_FRAMING_OK) {continue;}
        const auto * info = mavlink_get_message_info_by_id(message.msgid);
        if (!info) {++unknown_frames_; continue;}
        ++rx_frames_; last_rx_time_ = now; seen_rx_ = true;
        flapping_uav_msgs::msg::MavlinkFrame output;
        output.header.stamp = this->now();
        output.source_ip = ip; output.source_port = ntohs(peer.sin_port);
        output.wire_version = message.magic == MAVLINK_STX ? 2 : 1;
        output.system_id = message.sysid; output.component_id = message.compid;
        output.sequence = message.seq; output.message_id = message.msgid;
        output.message_name = info->name; output.fields_json = message_json(message);
        const auto * payload = reinterpret_cast<const uint8_t *>(_MAV_PAYLOAD(&message));
        output.payload.assign(payload, payload + message.len);
        rx_output_->publish(output);
        publish_attitude(message);
        publish_motion(message);
        if (print_rx_) {
          const auto key = source + "/" + std::to_string(message.sysid) + "/" +
            std::to_string(message.compid) + "/" + std::to_string(message.msgid);
          auto previous = last_print_.find(key);
          if (previous == last_print_.end() || now - previous->second >= std::chrono::seconds(1)) {
            if (last_print_.size() >= 1024) {last_print_.clear();}
            last_print_[key] = now;
            RCLCPP_INFO(
              get_logger(), "RX %s sys=%u comp=%u %s(%u) %s", source.c_str(),
              message.sysid, message.compid, info->name, message.msgid, output.fields_json.c_str());
          }
        }
      }
    }
  }

  void publish_status()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "mavlink_bridge"; status.hardware_id = remote_ip_;
    const bool rx_fresh = seen_rx_ && Clock::now() - last_rx_time_ < std::chrono::seconds(2);
    status.level = rx_fresh ? status.OK : status.WARN;
    status.message = rx_fresh ? "Receiving MAVLink" : "No fresh decoded MAVLink";
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value;
        status.values.push_back(item);
      };
    add("rx_port", std::to_string(rx_port_));
    add("destination", remote_ip_ + ":" + std::to_string(tx_port_));
    add("tx_enabled", tx_enabled_ ? "true" : "false");
    add("rc_input_fresh", rc_fresh() ? "true" : "false");
    add("rc_loss_action", loss_action_);
    add("tx_frames", std::to_string(tx_frames_)); add("rc_frames", std::to_string(rc_frames_));
    add("tx_errors", std::to_string(tx_errors_)); add("rx_errors", std::to_string(rx_errors_));
    add("rx_datagrams", std::to_string(rx_datagrams_));
    add("rx_frames", std::to_string(rx_frames_));
    add("attitude_order_drops", std::to_string(attitude_order_drops_));
    add("motion_order_drops", std::to_string(motion_order_drops_));
    add("attitude_clock_resets", std::to_string(attitude_clock_resets_));
    add("motion_clock_resets", std::to_string(motion_clock_resets_));
    add("crc_or_signature_errors", std::to_string(crc_errors_));
    add("unknown_frames", std::to_string(unknown_frames_));
    add("source_limit_drops", std::to_string(source_drops_));
    add("velocity_timeout_ms", std::to_string(velocity_timeout_ms_));
    add("velocity_attitude_timeout_ms", std::to_string(velocity_attitude_timeout_ms_));
    add("send_velocity", send_velocity_ ? "true" : "false");
    add("body_velocity_fresh", velocity_fresh() ? "true" : "false");
    add("velocity_coordinate_frame", "8 (see velocity_input_frame/application profile)");
    add("velocity_frames", std::to_string(velocity_frames_));
    add("velocity_input_frame", velocity_frame_id_);
    add("velocity_y_sign", std::to_string(velocity_y_sign_));
    add("velocity_xy_only", velocity_xy_only_ ? "true (mask 3559)" : "false (mask 3527)");
    add("motion_message", "250 DEBUG_VECT: x=forward m/s, y=right m/s, z=height up m");
    array.status.push_back(status); diagnostics_->publish(array);
  }

  SampleOrderFilter attitude_order_, motion_order_;
  uint64_t attitude_order_drops_{0}, motion_order_drops_{0};
  uint64_t attitude_clock_resets_{0}, motion_clock_resets_{0};
  UdpFd socket_;
  sockaddr_in remote_{};
  bool tx_enabled_{false}, send_heartbeat_{true}, print_rx_{true}, seen_rc_{false}, seen_rx_{false};
  int rx_port_, tx_port_, system_id_, component_id_, target_system_, target_component_,
    rc_timeout_ms_;
  std::string remote_ip_, loss_action_, motion_name_, velocity_frame_id_;
  Clock::time_point last_rc_time_{}, last_rx_time_{};
  Clock::time_point start_time_{Clock::now()}, last_velocity_time_{};
  bool send_velocity_{false}, seen_velocity_{false}, velocity_xy_only_{false};
  int velocity_y_sign_{1};
  int velocity_timeout_ms_, velocity_attitude_timeout_ms_, attitude_message_id_, attitude_system_, attitude_component_;
  double max_body_speed_;
  uint64_t velocity_frames_{0};
  flapping_uav_msgs::msg::BodyVelocitySetpoint last_velocity_;
  flapping_uav_msgs::msg::RcOverride last_rc_;
  std::map<std::string, Stream> streams_;
  std::map<std::string, Clock::time_point> last_print_;
  uint64_t tx_frames_{0}, rc_frames_{0}, tx_errors_{0}, rx_errors_{0}, rx_frames_{0};
  uint64_t rx_datagrams_{0}, crc_errors_{0}, unknown_frames_{0}, source_drops_{0};
  rclcpp::Publisher<flapping_uav_msgs::msg::MavlinkFrame>::SharedPtr rx_output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<flapping_uav_msgs::msg::RcOverride>::SharedPtr rc_input_;
  rclcpp::Publisher<flapping_uav_msgs::msg::AttitudeState>::SharedPtr attitude_output_;
  rclcpp::Publisher<flapping_uav_msgs::msg::BodyMotionState>::SharedPtr motion_output_;
  rclcpp::Publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>::SharedPtr sent_velocity_output_;
  rclcpp::Subscription<flapping_uav_msgs::msg::BodyVelocitySetpoint>::SharedPtr velocity_input_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_, tx_timer_, status_timer_;
};
rclcpp::Node::SharedPtr make_mavlink_bridge_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<MavlinkBridgeNode>(options);
}
}  // namespace flapping_uav_gcs

#ifndef FLAPPING_UAV_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(flapping_uav_gcs::make_mavlink_bridge_node(rclcpp::NodeOptions()));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mavlink_bridge_node"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
#endif
