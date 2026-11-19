#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>
#include <gtest/gtest.h>

#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_msgs/msg/mavlink_frame.hpp"
#include "flapping_uav_msgs/msg/rc_override.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "mavlink_codec.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace
{
class LoopSocket
{
public:
  LoopSocket()
  {
    fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {throw std::runtime_error("socket");}
    sockaddr_in address{}; address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      ::close(fd); throw std::runtime_error("bind");
    }
    socklen_t size = sizeof(address);
    ::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size);
    port = ntohs(address.sin_port);
  }
  ~LoopSocket() {if (fd >= 0) {::close(fd);}}
  void send(const std::vector<uint8_t> & bytes, int target)
  {
    sockaddr_in address{}; address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(target);
    ASSERT_EQ(
      ::sendto(
        fd, bytes.data(), bytes.size(), 0,
        reinterpret_cast<sockaddr *>(&address), sizeof(address)),
      static_cast<ssize_t>(bytes.size()));
  }
  int fd{-1}, port{};
};

std::vector<uint8_t> wire(const mavlink_message_t & message)
{
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes;
  const auto length = mavlink_msg_to_send_buffer(bytes.data(), &message);
  return {bytes.begin(), bytes.begin() + length};
}

class RcBridge : public ::testing::Test
{
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
};
}  // namespace

TEST_F(RcBridge, ActualDdsToUdp70AndInboundTelemetryWithStaleInputStop)
{
  LoopSocket board;
  int receive_port;
  {LoopSocket reserve; receive_port = reserve.port;}
  rclcpp::NodeOptions manual_options, bridge_options;
  manual_options.parameter_overrides(
    {rclcpp::Parameter("verify_device", false),
      rclcpp::Parameter("input_timeout_ms", 120)});
  bridge_options.parameter_overrides(
    {rclcpp::Parameter("rx_port", receive_port),
      rclcpp::Parameter("tx_port", board.port), rclcpp::Parameter("remote_ip", "127.0.0.1"),
      rclcpp::Parameter("tx_enabled", true), rclcpp::Parameter("rc_timeout_ms", 150),
      rclcpp::Parameter("print_rx", false)});
  auto manual = flapping_uav_gcs::make_manual_control_node(manual_options);
  auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(bridge_options);
  auto probe = std::make_shared<rclcpp::Node>("rc_bench_probe");
  auto joy = probe->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
  flapping_uav_msgs::msg::RcOverride last_rc;
  std::vector<flapping_uav_msgs::msg::MavlinkFrame> telemetry;
  auto rc_sub = probe->create_subscription<flapping_uav_msgs::msg::RcOverride>(
    "/rc/override", 10, [&last_rc](flapping_uav_msgs::msg::RcOverride::ConstSharedPtr msg) {
      last_rc = *msg;
    });
  auto frame_sub = probe->create_subscription<flapping_uav_msgs::msg::MavlinkFrame>(
    "/fc/mavlink_rx", 100, [&telemetry](flapping_uav_msgs::msg::MavlinkFrame::ConstSharedPtr msg) {
      telemetry.push_back(*msg);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manual); executor.add_node(bridge); executor.add_node(probe);
  std::vector<mavlink_message_t> outbound;
  mavlink_message_t tx_buffer{};
  mavlink_status_t tx_status{};
  auto step = [&]() {
      executor.spin_some();
      std::array<uint8_t, 2048> bytes;
      ssize_t length;
      while ((length = ::recv(board.fd, bytes.data(), bytes.size(), 0)) >= 0) {
        for (ssize_t i = 0; i < length; ++i) {
          mavlink_message_t message{}; mavlink_status_t status{};
          if (mavlink_frame_char_buffer(&tx_buffer, &tx_status, bytes[i], &message, &status) ==
            MAVLINK_FRAMING_OK) {outbound.push_back(message);}
        }
      }
      std::this_thread::sleep_for(2ms);
    };
  auto wait_until = [&](const std::function<bool()> & predicate, double duration) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
      while (!predicate() && std::chrono::steady_clock::now() < end) {step();}
      return predicate();
    };
  ASSERT_TRUE(wait_until([&]() {return joy->get_subscription_count() > 0;}, 3));
  sensor_msgs::msg::Joy sample;
  sample.axes = {1, 1, 1, -1, 1, -1, 0, -1};
  auto last_joy = std::chrono::steady_clock::now() - 1s;
  const auto end = std::chrono::steady_clock::now() + 1200ms;
  while (std::chrono::steady_clock::now() < end) {
    if (std::chrono::steady_clock::now() - last_joy > 30ms) {
      joy->publish(sample); last_joy = std::chrono::steady_clock::now();
    }
    step();
  }
  ASSERT_TRUE(last_rc.valid);
  EXPECT_EQ(last_rc.channels[2], 1000); EXPECT_EQ(last_rc.channels[3], 2000);
  int rc_messages = 0, heartbeats = 0;
  for (const auto & message : outbound) {
    EXPECT_EQ(message.magic, MAVLINK_STX);
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {++heartbeats; continue;}
    ASSERT_EQ(message.msgid, static_cast<uint32_t>(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE));
    ++rc_messages;
    mavlink_rc_channels_override_t decoded;
    mavlink_msg_rc_channels_override_decode(&message, &decoded);
    EXPECT_EQ(decoded.target_system, 1); EXPECT_EQ(decoded.target_component, 1);
    EXPECT_EQ(decoded.chan1_raw, 1000); EXPECT_EQ(decoded.chan2_raw, 1000);
    EXPECT_EQ(decoded.chan3_raw, 1000); EXPECT_EQ(decoded.chan4_raw, 2000);
    EXPECT_EQ(decoded.chan5_raw, 1000); EXPECT_EQ(decoded.chan6_raw, 2000);
    EXPECT_EQ(decoded.chan7_raw, 1500); EXPECT_EQ(decoded.chan8_raw, 2000);
    EXPECT_EQ(decoded.chan9_raw, 1000); EXPECT_EQ(decoded.chan11_raw, 1000);
    EXPECT_EQ(decoded.chan18_raw, 65535);
  }
  EXPECT_GE(rc_messages, 20); EXPECT_GE(heartbeats, 1);

  mavlink_message_t message;
  mavlink_msg_attitude_pack(1, 1, &message, 123, .1F, .2F, .3F, .4F, .5F, .6F);
  auto attitude = wire(message);
  board.send({attitude.begin(), attitude.begin() + 7}, receive_port);
  LoopSocket other_source;
  // A complete MAVLink 1 frame from another source between two fragments.
  auto * channel = mavlink_get_channel_status(MAVLINK_COMM_2);
  channel->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink_msg_heartbeat_pack_chan(2, 1, MAVLINK_COMM_2, &message, 2, 3, 0, 0, 3);
  other_source.send(wire(message), receive_port);
  channel->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  board.send({attitude.begin() + 7, attitude.end()}, receive_port);
  auto broken = attitude; broken.back() ^= 0x40;
  board.send(broken, receive_port);
  mavlink_msg_named_value_float_pack(1, 1, &message, 124, "height", NAN);
  board.send(wire(message), receive_port);
  const float q[4] = {1, 0, 0, 0};
  std::array<float, 21> covariance{}; covariance.fill(NAN);
  mavlink_msg_odometry_pack(
    1, 1, &message, 125, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD,
    NAN, NAN, -1, q, .2F, .3F, NAN, 0, 0, 0, covariance.data(), covariance.data(), 0, 0, -1);
  board.send(wire(message), receive_port);
  ASSERT_TRUE(wait_until([&]() {return telemetry.size() >= 4;}, 2));
  EXPECT_EQ(telemetry.size(), 4U);
  std::vector<uint32_t> ids;
  for (const auto & frame : telemetry) {
    ids.push_back(frame.message_id);
    EXPECT_EQ(frame.source_ip, "127.0.0.1");
    if (frame.message_id == 0) {EXPECT_EQ(frame.wire_version, 1);}
    if (frame.message_id == 251) {
      EXPECT_NE(frame.fields_json.find("\"value\":\"nan\""), std::string::npos);
    }
    if (frame.message_id == 331) {
      EXPECT_NE(frame.fields_json.find("\"quality\":-1"), std::string::npos);
    }
  }
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(ids, (std::vector<uint32_t>{0, 30, 251, 331}));

  ASSERT_TRUE(wait_until([&]() {return !last_rc.valid;}, 1));
  auto settle = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < settle) {step();}
  const auto count_rc = [&]() {
      return std::count_if(
        outbound.begin(), outbound.end(),
        [](const mavlink_message_t & msg) {return msg.msgid == 70;});
    };
  const auto count_before = count_rc();
  settle = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < settle) {step();}
  EXPECT_EQ(count_rc(), count_before);
  executor.remove_node(probe); executor.remove_node(bridge); executor.remove_node(manual);
}

TEST_F(RcBridge, RequiresExplicitDestinationForTransmission)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("tx_enabled", true),
      rclcpp::Parameter("remote_ip", "")});
  EXPECT_THROW(flapping_uav_gcs::make_mavlink_bridge_node(options), std::invalid_argument);
}

TEST_F(RcBridge, ReceiverOnlyNeverSendsAndRejectsRuntimeTxToggle)
{
  LoopSocket board;
  int receive_port;
  {LoopSocket reserve; receive_port = reserve.port;}
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("rx_port", receive_port),
      rclcpp::Parameter("tx_port", board.port), rclcpp::Parameter("remote_ip", "127.0.0.1"),
      rclcpp::Parameter("tx_enabled", false)});
  auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(options);
  const auto update = bridge->set_parameters_atomically({rclcpp::Parameter("tx_enabled", true)});
  EXPECT_FALSE(update.successful);
  auto probe = std::make_shared<rclcpp::Node>("receiver_only_probe");
  auto publisher = probe->create_publisher<flapping_uav_msgs::msg::RcOverride>("/rc/override", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(probe);
  flapping_uav_msgs::msg::RcOverride rc;
  rc.valid = true; rc.device_connected = true; rc.input_age_ms = 0;
  rc.channels.fill(1500);
  auto end = std::chrono::steady_clock::now() + 1200ms;
  while (std::chrono::steady_clock::now() < end) {
    publisher->publish(rc); executor.spin_some(); std::this_thread::sleep_for(10ms);
  }
  std::array<uint8_t, 2048> data;
  EXPECT_LT(::recv(board.fd, data.data(), data.size(), 0), 0);
  executor.remove_node(probe); executor.remove_node(bridge);
}
