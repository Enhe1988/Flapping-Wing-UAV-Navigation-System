#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavlink_codec.hpp"

using namespace std::chrono_literals;  // NOLINT
using flapping_uav_gcs::map_velocity_to_body;
using flapping_uav_gcs::Velocity3;

TEST(AlignmentSession, MatchesOnlyBoundHostAndContinuousFcBoot)
{
  flapping_uav_gcs::AlignmentSessionConfig c;
  c.required=true;c.device_time_us=500000000;c.host_monotonic_ns=1000000000000LL;
  c.host_boot_id="host-A";
  EXPECT_TRUE(flapping_uav_gcs::alignment_matches_session(c,"host-A",502000000,1002000000000LL));
  EXPECT_TRUE(flapping_uav_gcs::alignment_matches_session(c,"host-A",502000000,1002200000000LL));
  EXPECT_FALSE(flapping_uav_gcs::alignment_matches_session(c,"host-B",502000000,1002000000000LL));
  EXPECT_FALSE(flapping_uav_gcs::alignment_matches_session(c,"host-A",2000000,1002000000000LL));
  // A reboot is also detected if new uptime eventually exceeds the saved uptime.
  EXPECT_FALSE(flapping_uav_gcs::alignment_matches_session(c,"host-A",600000000,1600000000000LL));
}

TEST(AlignmentSession, RejectsMissingBindingAndStaleReference)
{
  flapping_uav_gcs::AlignmentSessionConfig c;c.required=true;
  EXPECT_FALSE(flapping_uav_gcs::alignment_matches_session(c,"host-A",1,1));
  c.device_time_us=1000000;c.host_monotonic_ns=100000000000LL;c.host_boot_id="host-A";
  EXPECT_FALSE(flapping_uav_gcs::alignment_matches_session(c,"host-A",3000000,103500000000LL));
  EXPECT_FALSE(flapping_uav_gcs::alignment_matches_session(c,"host-A",3000000,99000000000LL));
}

TEST(AlignmentSession, GenericUnboundFixtureRemainsAvailable)
{
  flapping_uav_gcs::AlignmentSessionConfig c;
  EXPECT_TRUE(flapping_uav_gcs::alignment_matches_session(c,"",0,0));
}

TEST(VelocityRotation, FrameAxesYawCalibrationAndQuaternionDirection)
{
  Velocity3 output;
  const double half = std::sqrt(0.5);
  ASSERT_TRUE(map_velocity_to_body({{1, 0, 0}}, 0, {{1, 0, 0, 0}}, true, output));
  EXPECT_NEAR(output[0], 0, 1e-12); EXPECT_NEAR(output[1], 1, 1e-12);
  ASSERT_TRUE(map_velocity_to_body({{1, 0, 0}}, 0, {{half, 0, 0, half}}, true, output));
  EXPECT_NEAR(output[0], 1, 1e-12); EXPECT_NEAR(output[1], 0, 1e-12);
  ASSERT_TRUE(map_velocity_to_body({{0, 1, 0}}, 0, {{half, 0, 0, half}}, true, output));
  EXPECT_NEAR(output[0], 0, 1e-12); EXPECT_NEAR(output[1], -1, 1e-12);
  ASSERT_TRUE(map_velocity_to_body({{1, 0, 0}}, 0, {{half, 0, 0, -half}}, false, output));
  EXPECT_NEAR(output[0], 1, 1e-12);
  ASSERT_TRUE(
    map_velocity_to_body(
      {{1, 0, 0}}, std::acos(-1) / 2,
      {{1, 0, 0, 0}}, true, output));
  EXPECT_NEAR(output[0], 1, 1e-12); EXPECT_NEAR(output[1], 0, 1e-12);
}

TEST(VelocityRotation, FullTiltPreservesNormAndDoesNotForceBodyZToZero)
{
  const double half = std::sqrt(0.5);
  Velocity3 output;
  ASSERT_TRUE(map_velocity_to_body({{0, 1, 0}}, 0, {{half, 0, half, 0}}, true, output));
  EXPECT_NEAR(output[0], 0, 1e-12); EXPECT_NEAR(output[1], 0, 1e-12);
  EXPECT_NEAR(output[2], 1, 1e-12);
  ASSERT_TRUE(map_velocity_to_body({{0, 0, 1}}, 0, {{1, 0, 0, 0}}, true, output));
  EXPECT_NEAR(output[2], -1, 1e-12);
  ASSERT_TRUE(
    map_velocity_to_body(
      {{.2, -.3, .4}}, .4,
      {{.5, .5, .5, .5}}, true, output));
  EXPECT_NEAR(std::hypot(output[0], output[1], output[2]), std::sqrt(.29), 1e-12);
  EXPECT_FALSE(map_velocity_to_body({{1, 0, 0}}, 0, {{0, 0, 0, 0}}, true, output));
  EXPECT_FALSE(map_velocity_to_body({{1, 0, 0}}, 0, {{2, 0, 0, 0}}, true, output));
  EXPECT_FALSE(map_velocity_to_body({{1, 0, 0}}, 0, {{NAN, 0, 0, 0}}, true, output));
  EXPECT_FALSE(map_velocity_to_body({{NAN, 0, 0}}, 0, {{1, 0, 0, 0}}, true, output));
}

namespace
{
class WirePeer
{
public:
  WirePeer()
  {
    fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {throw std::runtime_error("socket");}
    sockaddr_in local{}; local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
      ::close(fd); throw std::runtime_error("bind");
    }
    socklen_t size = sizeof(local);
    ::getsockname(fd, reinterpret_cast<sockaddr *>(&local), &size);
    port = ntohs(local.sin_port);
  }
  ~WirePeer() {::close(fd);}
  void attitude(int destination, uint32_t time_ms, bool valid)
  {
    mavlink_message_t message;
    const float half = std::sqrt(0.5F);
    const float display_offset[4] = {0, 0, 0, 0};
    mavlink_msg_attitude_quaternion_pack(
      1, 1, &message, time_ms,
      valid ? half : NAN, 0, 0, half, 0, 0, 0, display_offset);
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer;
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    sockaddr_in target{}; target.sin_family = AF_INET;
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK); target.sin_port = htons(destination);
    ASSERT_EQ(
      ::sendto(
        fd, buffer.data(), length, 0,
        reinterpret_cast<sockaddr *>(&target), sizeof(target)), length);
  }
  int fd, port;
};
class VelocityGraph : public ::testing::Test
{
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
};
}  // namespace

TEST_F(VelocityGraph, RealTelemetryToComputerRotationToBody84AndInvalidation)
{
  WirePeer board;
  int rx_port;
  {WirePeer temporary; rx_port = temporary.port;}
  rclcpp::NodeOptions bridge_options, transform_options;
  bridge_options.parameter_overrides(
    {rclcpp::Parameter("rx_port", rx_port),
      rclcpp::Parameter("tx_port", board.port), rclcpp::Parameter("remote_ip", "127.0.0.1"),
      rclcpp::Parameter("tx_enabled", true), rclcpp::Parameter("send_velocity", true),
      rclcpp::Parameter("send_heartbeat", false), rclcpp::Parameter("print_rx", false),
      rclcpp::Parameter("velocity_timeout_ms", 160)});
  transform_options.parameter_overrides(
    {rclcpp::Parameter("map_to_reference_confirmed", true),
      rclcpp::Parameter("attitude_convention_confirmed", true),
      rclcpp::Parameter("command_timeout_ms", 120), rclcpp::Parameter("attitude_timeout_ms", 120)});
  auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(bridge_options);
  auto transform = flapping_uav_gcs::make_velocity_transform_node(transform_options);
  auto probe = std::make_shared<rclcpp::Node>("body_velocity_probe");
  auto command = probe->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/navigation/velocity_setpoint_map", 10);
  flapping_uav_msgs::msg::BodyVelocitySetpoint last;
  auto subscription = probe->create_subscription<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
    "/fc/velocity_setpoint_body", 10,
    [&last](flapping_uav_msgs::msg::BodyVelocitySetpoint::ConstSharedPtr msg) {last = *msg;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(transform); executor.add_node(probe);
  std::vector<mavlink_set_position_target_local_ned_t> sent;
  mavlink_message_t buffer{}; mavlink_status_t parser{};
  auto step = [&]() {
      executor.spin_some();
      std::array<uint8_t, 2048> data;
      ssize_t length;
      while ((length = ::recv(board.fd, data.data(), data.size(), 0)) >= 0) {
        for (ssize_t i = 0; i < length; ++i) {
          mavlink_message_t message{}; mavlink_status_t status{};
          if (mavlink_frame_char_buffer(&buffer, &parser, data[i], &message, &status) !=
            MAVLINK_FRAMING_OK) {continue;}
          EXPECT_EQ(message.msgid, 84U);
          if (message.msgid != 84) {continue;}
          mavlink_set_position_target_local_ned_t decoded;
          mavlink_msg_set_position_target_local_ned_decode(&message, &decoded);
          EXPECT_EQ(decoded.coordinate_frame, 8); EXPECT_EQ(decoded.type_mask, 3527);
          EXPECT_EQ(decoded.target_system, 1); EXPECT_EQ(decoded.target_component, 1);
          sent.push_back(decoded);
        }
      }
      std::this_thread::sleep_for(3ms);
    };
  geometry_msgs::msg::TwistStamped world;
  world.header.frame_id = "map"; world.twist.linear.x = .4;
  uint32_t sample_time = 100;
  auto run = [&](double seconds, bool publish_command, bool valid_pose, bool frozen_pose) {
      auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      auto next = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() < end) {
        if (std::chrono::steady_clock::now() >= next) {
          if (publish_command) {command->publish(world);}
          if (!frozen_pose) {sample_time += 20;}
          board.attitude(rx_port, sample_time, valid_pose);
          next += 20ms;
        }
        step();
      }
    };
  run(1.0, true, true, false);
  ASSERT_TRUE(last.valid) << last.reason;
  EXPECT_NEAR(last.velocity.x, .4, 1e-6); EXPECT_NEAR(last.velocity.y, 0, 1e-6);
  ASSERT_FALSE(sent.empty());
  EXPECT_NEAR(sent.back().vx, .4, 1e-6); EXPECT_NEAR(sent.back().vy, 0, 1e-6);
  run(.3, false, true, false);
  EXPECT_FALSE(last.valid); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  run(.2, true, true, false); ASSERT_TRUE(last.valid);
  run(.15, true, false, false);
  EXPECT_FALSE(last.valid); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  run(.2, true, true, false); ASSERT_TRUE(last.valid);
  run(.3, true, true, true);
  EXPECT_FALSE(last.valid); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  run(.2, true, true, false); ASSERT_TRUE(last.valid);
  world.header.frame_id = "local_ned";
  run(.15, true, true, false);
  EXPECT_FALSE(last.valid); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  executor.remove_node(probe); executor.remove_node(transform); executor.remove_node(bridge);
}

TEST_F(VelocityGraph, UnverifiedConventionsProduceOnlyInvalidZeroSetpoints)
{
  auto transform = flapping_uav_gcs::make_velocity_transform_node(rclcpp::NodeOptions());
  auto probe = std::make_shared<rclcpp::Node>("unverified_transform_probe");
  flapping_uav_msgs::msg::BodyVelocitySetpoint last;
  unsigned received = 0;
  auto sub = probe->create_subscription<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
    "/fc/velocity_setpoint_body", 10,
    [&](flapping_uav_msgs::msg::BodyVelocitySetpoint::ConstSharedPtr msg) {
      last = *msg; ++received;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transform); executor.add_node(probe);
  auto end = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < end) {
    executor.spin_some(); std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(received, 0U);
  EXPECT_FALSE(last.valid); EXPECT_EQ(last.reason, "Map/ENU/NED alignment unconfirmed");
  EXPECT_DOUBLE_EQ(last.velocity.x, 0); EXPECT_DOUBLE_EQ(last.velocity.y, 0);
  EXPECT_DOUBLE_EQ(last.velocity.z, 0);
  executor.remove_node(probe); executor.remove_node(transform);
}

TEST_F(VelocityGraph, SlowerAttitudeDoesNotExtendCommandWatchdog)
{
  WirePeer board;
  int rx; {WirePeer reserve; rx = reserve.port;}
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rx_port", rx),
    rclcpp::Parameter("tx_port", board.port), rclcpp::Parameter("remote_ip", "127.0.0.1"),
    rclcpp::Parameter("tx_enabled", true), rclcpp::Parameter("send_velocity", true),
    rclcpp::Parameter("send_heartbeat", false), rclcpp::Parameter("print_rx", false),
    rclcpp::Parameter("velocity_timeout_ms", 200),
    rclcpp::Parameter("velocity_attitude_timeout_ms", 1500)});
  auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(options);
  auto probe = std::make_shared<rclcpp::Node>("separate_watchdog_probe");
  auto command = probe->create_publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
    "/fc/velocity_setpoint_body", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(probe);
  float last_x = NAN;
  mavlink_message_t buffer{}; mavlink_status_t parser{};
  auto run = [&](double seconds, bool feed, double command_age, double attitude_age) {
      const auto end = std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now()<end) {
        if (feed) {
          flapping_uav_msgs::msg::BodyVelocitySetpoint m;
          m.header.frame_id = "base_link_frd"; m.header.stamp = probe->now(); m.valid = true;
          m.velocity.x = .1; m.command_age_ms = command_age; m.attitude_age_ms = attitude_age;
          command->publish(m);
        }
        executor.spin_some();
        std::array<uint8_t, 2048> bytes; ssize_t count;
        while ((count = ::recv(board.fd, bytes.data(), bytes.size(), 0))>=0) {
          for (ssize_t i=0; i<count; ++i) {
            mavlink_message_t m{}; mavlink_status_t status{};
            if (mavlink_frame_char_buffer(&buffer,&parser,bytes[i],&m,&status)==MAVLINK_FRAMING_OK &&
              m.msgid==84) {
              mavlink_set_position_target_local_ned_t v;
              mavlink_msg_set_position_target_local_ned_decode(&m,&v); last_x=v.vx;
            }
          }
        }
        std::this_thread::sleep_for(5ms);
      }
    };
  run(.6, true, 0, 1000); EXPECT_NEAR(last_x, .1, 1e-6);
  run(.15, true, 0, 1600); EXPECT_FLOAT_EQ(last_x, 0);
  run(.15, true, 250, 0); EXPECT_FLOAT_EQ(last_x, 0);
  run(.15, true, 0, 1000); EXPECT_NEAR(last_x, .1, 1e-6);
  run(.3, false, 0, 0); EXPECT_FLOAT_EQ(last_x, 0);
  executor.remove_node(probe); executor.remove_node(bridge);
}

TEST_F(VelocityGraph, FinalWireYFlipPreservesXFeedbackAndStaleZero)
{
  WirePeer board;
  int rx; {WirePeer reserve; rx=reserve.port;}
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rx_port",rx),
    rclcpp::Parameter("tx_port",board.port),rclcpp::Parameter("remote_ip","127.0.0.1"),
    rclcpp::Parameter("tx_enabled",true),rclcpp::Parameter("send_velocity",true),
    rclcpp::Parameter("send_heartbeat",false),rclcpp::Parameter("print_rx",false),
    rclcpp::Parameter("velocity_xy_only",true),rclcpp::Parameter("heading_xy_profile_confirmed",true),
    rclcpp::Parameter("velocity_frame_id","base_link_heading_frd"),
    rclcpp::Parameter("velocity_y_sign",-1)});
  auto bridge=flapping_uav_gcs::make_mavlink_bridge_node(options);
  auto probe=std::make_shared<rclcpp::Node>("wire_y_sign_probe");
  auto command=probe->create_publisher<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
    "/fc/velocity_setpoint_body",10);
  flapping_uav_msgs::msg::BodyVelocitySetpoint monitor;
  flapping_uav_msgs::msg::BodyMotionState feedback;
  unsigned monitor_count=0, feedback_count=0;
  auto ms=probe->create_subscription<flapping_uav_msgs::msg::BodyVelocitySetpoint>(
    "/fc/velocity_setpoint_sent",10,
    [&](flapping_uav_msgs::msg::BodyVelocitySetpoint::ConstSharedPtr m){monitor=*m;++monitor_count;});
  auto fs=probe->create_subscription<flapping_uav_msgs::msg::BodyMotionState>(
    "/fc/body_motion",10,
    [&](flapping_uav_msgs::msg::BodyMotionState::ConstSharedPtr m){feedback=*m;++feedback_count;});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(bridge);executor.add_node(probe);
  mavlink_message_t buffer{};mavlink_status_t parser{};
  mavlink_set_position_target_local_ned_t wire{};unsigned wire_count=0;
  bool valid=true;double y=.2;uint64_t sample_us=1000000;
  auto run=[&](double seconds,bool feed_command){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    auto next=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()<end){
      if(std::chrono::steady_clock::now()>=next){
        if(feed_command){
          flapping_uav_msgs::msg::BodyVelocitySetpoint m;m.header.stamp=probe->now();
          m.header.frame_id="base_link_heading_frd";m.valid=valid;
          m.command_age_ms=m.attitude_age_ms=0;m.velocity.x=.12;m.velocity.y=y;command->publish(m);
        }
        // Receive a positive-right ID250 independently of the TX sign setting.
        mavlink_message_t motion;const char name[10]="flow";
        mavlink_msg_debug_vect_pack(1,1,&motion,name,sample_us,.03F,.07F,.4F);sample_us+=20000;
        std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes;
        auto length=mavlink_msg_to_send_buffer(bytes.data(),&motion);
        sockaddr_in target{};target.sin_family=AF_INET;target.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        target.sin_port=htons(rx);
        ASSERT_EQ(::sendto(board.fd,bytes.data(),length,0,reinterpret_cast<sockaddr *>(&target),sizeof(target)),length);
        next+=20ms;
      }
      executor.spin_some();std::array<uint8_t,2048> bytes;ssize_t length;
      while((length=::recv(board.fd,bytes.data(),bytes.size(),0))>=0){
        for(ssize_t i=0;i<length;++i){
          mavlink_message_t m{};mavlink_status_t status{};
          if(mavlink_frame_char_buffer(&buffer,&parser,bytes[i],&m,&status)!=MAVLINK_FRAMING_OK){continue;}
          ASSERT_EQ(m.msgid,84U);mavlink_msg_set_position_target_local_ned_decode(&m,&wire);++wire_count;
          EXPECT_EQ(wire.coordinate_frame,8);EXPECT_EQ(wire.type_mask,3559);EXPECT_FLOAT_EQ(wire.vz,0);
        }
      }
      std::this_thread::sleep_for(2ms);
    }
  };
  run(.7,true);ASSERT_GT(wire_count,0U);ASSERT_GT(monitor_count,0U);ASSERT_GT(feedback_count,0U);
  EXPECT_FLOAT_EQ(wire.vx,.12F);EXPECT_FLOAT_EQ(wire.vy,-.2F);
  EXPECT_TRUE(monitor.valid);EXPECT_EQ(monitor.header.frame_id,"fc_velocity_command");
  EXPECT_DOUBLE_EQ(monitor.velocity.x,wire.vx);EXPECT_DOUBLE_EQ(monitor.velocity.y,wire.vy);
  EXPECT_NE(monitor.reason.find("vy sign=-1"),std::string::npos);
  EXPECT_NEAR(feedback.right_velocity_mps,.07,1e-6);EXPECT_TRUE(feedback.velocity_valid);
  y=-.2;run(.15,true);EXPECT_FLOAT_EQ(wire.vx,.12F);EXPECT_FLOAT_EQ(wire.vy,.2F);
  valid=false;run(.15,true);EXPECT_FLOAT_EQ(wire.vx,0);EXPECT_FLOAT_EQ(wire.vy,0);
  EXPECT_FALSE(monitor.valid);EXPECT_DOUBLE_EQ(monitor.velocity.y,0);
  valid=true;run(.1,true);ASSERT_FLOAT_EQ(wire.vy,.2F);
  run(.35,false);EXPECT_FLOAT_EQ(wire.vx,0);EXPECT_FLOAT_EQ(wire.vy,0);EXPECT_FALSE(monitor.valid);
  executor.remove_node(probe);executor.remove_node(bridge);
}
