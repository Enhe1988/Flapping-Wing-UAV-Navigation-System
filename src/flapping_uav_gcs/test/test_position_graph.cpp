#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <future>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "flapping_uav_msgs/msg/navigation_status.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mavlink_codec.hpp"

using namespace std::chrono_literals;  // NOLINT
namespace
{
class LoopPeer
{
public:
  LoopPeer()
  {
    fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {throw std::runtime_error("socket");}
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) {
      ::close(fd); throw std::runtime_error("bind");
    }
    socklen_t size = sizeof(a); ::getsockname(fd, reinterpret_cast<sockaddr *>(&a), &size);
    port = ntohs(a.sin_port);
  }
  ~LoopPeer() {::close(fd);}
  void send(const mavlink_message_t & m, int port)
  {
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes;
    const auto size = mavlink_msg_to_send_buffer(bytes.data(), &m);
    sockaddr_in a{}; a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK); a.sin_port = htons(port);
    ASSERT_EQ(::sendto(fd, bytes.data(), size, 0, reinterpret_cast<sockaddr *>(&a), sizeof(a)), size);
  }
  int fd, port;
};
class PositionGraph : public ::testing::Test
{
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
};
}  // namespace

TEST_F(PositionGraph, Telemetry250ToPdTo84ArrivalCancelAndIndependentInputLoss)
{
  LoopPeer board;
  int rx; {LoopPeer reserve; rx = reserve.port;}
  rclcpp::NodeOptions bo, vo, co;
  bo.parameter_overrides({rclcpp::Parameter("rx_port", rx), rclcpp::Parameter("tx_port", board.port),
    rclcpp::Parameter("remote_ip", "127.0.0.1"), rclcpp::Parameter("tx_enabled", true),
    rclcpp::Parameter("send_velocity", true), rclcpp::Parameter("velocity_xy_only", true),
    rclcpp::Parameter("velocity_frame_id", "base_link_heading_frd"),
    rclcpp::Parameter("heading_xy_profile_confirmed", true),
    rclcpp::Parameter("send_heartbeat", false), rclcpp::Parameter("print_rx", false),
    rclcpp::Parameter("motion_name", "motion")});
  vo.parameter_overrides({rclcpp::Parameter("map_to_reference_confirmed", true),
    rclcpp::Parameter("rotation_mode", "heading_only"),
    rclcpp::Parameter("body_frame_id", "base_link_heading_frd"),
    rclcpp::Parameter("attitude_convention_confirmed", true), rclcpp::Parameter("attitude_timeout_ms", 150)});
  // Synthetic test fixture, not flight tuning.
  co.parameter_overrides({rclcpp::Parameter("map_to_reference_confirmed", true),
    rclcpp::Parameter("attitude_convention_confirmed", true),
    rclcpp::Parameter("feedback_convention_confirmed", true), rclcpp::Parameter("feedback_velocity_mode", "heading_xy"),
    rclcpp::Parameter("kp_xy", .5), rclcpp::Parameter("kd_xy", .2), rclcpp::Parameter("max_speed_mps", .3),
    rclcpp::Parameter("max_acceleration_mps2", .5), rclcpp::Parameter("arrival_position_tolerance_m", .05),
    rclcpp::Parameter("arrival_speed_tolerance_mps", .05), rclcpp::Parameter("arrival_dwell_s", .12),
    rclcpp::Parameter("position_timeout_ms", 150), rclcpp::Parameter("feedback_timeout_ms", 150),
    rclcpp::Parameter("attitude_timeout_ms", 150), rclcpp::Parameter("heartbeat_timeout_ms", 200)});
  timespec boot_reference{};ASSERT_EQ(::clock_gettime(CLOCK_MONOTONIC,&boot_reference),0);
  const int64_t reference_ns=static_cast<int64_t>(boot_reference.tv_sec)*1000000000LL+boot_reference.tv_nsec;
  std::string host_boot;{std::ifstream file("/proc/sys/kernel/random/boot_id");std::getline(file,host_boot);}
  ASSERT_FALSE(host_boot.empty());
  for (auto * options : {&co,&vo}) {
    auto parameters=options->parameter_overrides();
    parameters.push_back(rclcpp::Parameter("alignment_requires_session",true));
    parameters.push_back(rclcpp::Parameter("alignment_device_time_us",int64_t(100000)));
    parameters.push_back(rclcpp::Parameter("alignment_host_monotonic_ns",reference_ns));
    parameters.push_back(rclcpp::Parameter("alignment_host_boot_id",host_boot));
    options->parameter_overrides(parameters);
  }
  auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(bo);
  auto transform = flapping_uav_gcs::make_velocity_transform_node(vo);
  auto control = flapping_uav_gcs::make_position_controller_node(co);
  auto probe = std::make_shared<rclcpp::Node>("position_graph_probe");
  auto pose = probe->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position", 10);
  auto uwb = probe->create_publisher<flapping_uav_msgs::msg::UwbStatus>("/uwb/status", 10);
  auto goal = probe->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  auto enable = probe->create_client<std_srvs::srv::SetBool>("/navigation/enable");
  auto cancel = probe->create_client<std_srvs::srv::Trigger>("/navigation/cancel_goal");
  flapping_uav_msgs::msg::NavigationStatus last;
  flapping_uav_msgs::msg::BodyMotionState motion;
  unsigned motion_count = 0;
  auto ss = probe->create_subscription<flapping_uav_msgs::msg::NavigationStatus>("/navigation/status", 10,
    [&](flapping_uav_msgs::msg::NavigationStatus::ConstSharedPtr m) {last = *m;});
  auto ms = probe->create_subscription<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion", 10,
    [&](flapping_uav_msgs::msg::BodyMotionState::ConstSharedPtr m) {motion = *m; ++motion_count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto node : {bridge, transform, control, probe}) {executor.add_node(node);}
  std::vector<mavlink_set_position_target_local_ned_t> sent;
  mavlink_message_t buffer{}; mavlink_status_t parser{};
  auto step = [&]() {
      executor.spin_some(); std::array<uint8_t, 4096> bytes; ssize_t length;
      while ((length = ::recv(board.fd, bytes.data(), bytes.size(), 0)) >= 0) {
        for (ssize_t i = 0; i < length; ++i) {
          mavlink_message_t m{}; mavlink_status_t s{};
          if (mavlink_frame_char_buffer(&buffer, &parser, bytes[i], &m, &s) != MAVLINK_FRAMING_OK) {continue;}
          ASSERT_EQ(m.msgid, 84U);
          mavlink_set_position_target_local_ned_t v; mavlink_msg_set_position_target_local_ned_decode(&m, &v);
          EXPECT_EQ(v.coordinate_frame, 8); EXPECT_EQ(v.type_mask, 3559);
          EXPECT_FLOAT_EQ(v.vz, 0); EXPECT_TRUE(std::isfinite(v.vx)); sent.push_back(v);
        }
      }
      std::this_thread::sleep_for(2ms);
    };
  bool fp = true, fm = true, fa = true, fh = true, freeze = false, bad = false, valid_uwb = true;
  double actual_x = 0, actual_v = 0;
  float height = 1.2F;
  uint32_t tm = 100, mt = 100;
  const char name[10] = "motion"; const float offset[4] = {0, 0, 0, 0};
  auto run = [&](double seconds) {
      auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      auto next = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() < end) {
        if (std::chrono::steady_clock::now() >= next) {
          tm += 20; if (!freeze) {mt += 20;}
          mavlink_message_t m;
          if (fh) {mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE); board.send(m, rx);}
          if (fa) {
            const float h = std::sqrt(.5F), cp = std::cos(std::acos(-1.F)/12), sp = std::sin(std::acos(-1.F)/12);
            // Yaw90 and pitch30: planar setpoints must NOT shrink by cos(pitch).
            mavlink_msg_attitude_quaternion_pack(1, 1, &m, tm, h*cp, -h*sp, h*sp, h*cp, 0, 0, 0, offset); board.send(m, rx);
          }
          if (fm) {mavlink_msg_debug_vect_pack(1, 1, &m, name, static_cast<uint64_t>(mt)*1000, bad ? NAN : actual_v, 0, height); board.send(m, rx);}
          if (fp) {
            geometry_msgs::msg::PoseWithCovarianceStamped p; p.header.stamp = probe->now(); p.header.frame_id = "map";
            p.pose.pose.position.x = actual_x; p.pose.pose.position.z = 999; pose->publish(p);
          }
          flapping_uav_msgs::msg::UwbStatus s; s.header.stamp = probe->now(); s.header.frame_id = "map";
          s.position_valid = valid_uwb; s.coordinates_confirmed = true; uwb->publish(s); next += 20ms;
        }
        step();
      }
    };
  auto new_goal = [&]() {geometry_msgs::msg::PoseStamped g; g.header.frame_id = "map"; g.header.stamp = probe->now(); g.pose.position.x = .8; g.pose.position.z = 123; goal->publish(g);};
  run(1.0); ASSERT_TRUE(last.inputs_valid) << last.reason;
  EXPECT_FALSE(last.enabled); ASSERT_GT(motion_count, 0U); EXPECT_TRUE(motion.velocity_valid); EXPECT_TRUE(motion.height_valid);
  EXPECT_NEAR(motion.height_up_m, 1.2, 1e-6); ASSERT_FALSE(sent.empty()); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); request->data = true;
  auto enabled = enable->async_send_request(request); run(.2);
  ASSERT_EQ(enabled.wait_for(0s), std::future_status::ready); ASSERT_TRUE(enabled.get()->success);
  new_goal(); run(.7); ASSERT_EQ(last.state, "ACTIVE"); EXPECT_GT(last.velocity_map.x, .1);
  EXPECT_GT(sent.back().vx, .1); EXPECT_NEAR(sent.back().vy, 0, 1e-5); EXPECT_LE(std::hypot(sent.back().vx, sent.back().vy), .30001);
  height = NAN; run(.1); EXPECT_TRUE(motion.velocity_valid); EXPECT_FALSE(motion.height_valid); EXPECT_EQ(last.state, "ACTIVE");
  height = 1.5F; actual_x = .8; actual_v = .2; run(.25); EXPECT_FALSE(last.reached);
  actual_v = 0; run(.3); ASSERT_TRUE(last.reached); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  actual_x = 0; new_goal(); run(.3); EXPECT_GT(sent.back().vx, .05);
  auto cancelled = cancel->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()); run(.15);
  ASSERT_EQ(cancelled.wait_for(0s), std::future_status::ready); EXPECT_TRUE(cancelled.get()->success); EXPECT_FALSE(last.goal_active); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  for (int failure = 0; failure < 6; ++failure) {
    new_goal(); run(.3); ASSERT_GT(sent.back().vx, .04) << failure;
    fp = failure != 0; fm = failure != 1; fa = failure != 2; fh = failure != 3;
    freeze = failure == 4; valid_uwb = failure != 5; run(.3);
    EXPECT_EQ(last.state, "FAULT") << failure; EXPECT_FALSE(last.goal_active); EXPECT_FLOAT_EQ(sent.back().vx, 0);
    fp = fm = fa = fh = valid_uwb = true; freeze = false; run(.2);
    EXPECT_FALSE(last.goal_active); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  }
  new_goal(); run(.3); bad = true; run(.15); EXPECT_EQ(last.state, "FAULT"); EXPECT_FLOAT_EQ(sent.back().vx, 0);
  bad=false;run(.2);new_goal();run(.3);ASSERT_GT(sent.back().vx,.04);
  // New FC boot: fresh packets alone must not make the old alignment reusable.
  tm=0;mt=0;run(.3);
  EXPECT_EQ(last.state,"FAULT");EXPECT_FLOAT_EQ(sent.back().vx,0);
  auto reenable=enable->async_send_request(request);run(.15);
  ASSERT_EQ(reenable.wait_for(0s),std::future_status::ready);
  EXPECT_FALSE(reenable.get()->success);
  for (auto node : {probe, control, transform, bridge}) {executor.remove_node(node);}
}

TEST_F(PositionGraph, StartHereCapturesHeadingAndCurrentGoalThenHoldsAndInvalidatesOnReboot)
{
  LoopPeer board;
  int rx; {LoopPeer reserve; rx = reserve.port;}
  rclcpp::NodeOptions bo, vo, co;
  bo.parameter_overrides({rclcpp::Parameter("rx_port", rx), rclcpp::Parameter("tx_port", board.port),
    rclcpp::Parameter("remote_ip", "127.0.0.1"), rclcpp::Parameter("tx_enabled", true),
    rclcpp::Parameter("send_velocity", true), rclcpp::Parameter("velocity_xy_only", true),
    rclcpp::Parameter("velocity_frame_id", "base_link_heading_frd"),
    rclcpp::Parameter("heading_xy_profile_confirmed", true),
    rclcpp::Parameter("send_heartbeat", false), rclcpp::Parameter("print_rx", false),
    rclcpp::Parameter("motion_name", "motion")});
  vo.parameter_overrides({rclcpp::Parameter("map_to_reference_confirmed", true),
    rclcpp::Parameter("rotation_mode", "heading_only"),
    rclcpp::Parameter("body_frame_id", "base_link_heading_frd"),
    rclcpp::Parameter("attitude_convention_confirmed", true), rclcpp::Parameter("attitude_timeout_ms", 150)});
  // Synthetic test fixture, not flight tuning.
  co.parameter_overrides({rclcpp::Parameter("map_to_reference_confirmed", true),
    rclcpp::Parameter("attitude_convention_confirmed", true),
    rclcpp::Parameter("feedback_convention_confirmed", true), rclcpp::Parameter("feedback_velocity_mode", "heading_xy"),
    rclcpp::Parameter("kp_xy", .5), rclcpp::Parameter("kd_xy", .2), rclcpp::Parameter("max_speed_mps", .3),
    rclcpp::Parameter("max_acceleration_mps2", .5), rclcpp::Parameter("arrival_position_tolerance_m", .05),
    rclcpp::Parameter("arrival_speed_tolerance_mps", .05), rclcpp::Parameter("arrival_dwell_s", .12),
    rclcpp::Parameter("position_timeout_ms", 150), rclcpp::Parameter("feedback_timeout_ms", 150),
    rclcpp::Parameter("attitude_timeout_ms", 150), rclcpp::Parameter("heartbeat_timeout_ms", 200)});
  timespec boot_reference{};ASSERT_EQ(::clock_gettime(CLOCK_MONOTONIC,&boot_reference),0);
  const int64_t reference_ns=static_cast<int64_t>(boot_reference.tv_sec)*1000000000LL+boot_reference.tv_nsec;
  std::string host_boot;{std::ifstream file("/proc/sys/kernel/random/boot_id");std::getline(file,host_boot);}
  ASSERT_FALSE(host_boot.empty());
  for (auto * options : {&co,&vo}) {
    auto parameters=options->parameter_overrides();
    parameters.push_back(rclcpp::Parameter("alignment_requires_session",true));
    parameters.push_back(rclcpp::Parameter("alignment_device_time_us",int64_t(100000)));
    parameters.push_back(rclcpp::Parameter("alignment_host_monotonic_ns",reference_ns));
    parameters.push_back(rclcpp::Parameter("alignment_host_boot_id",host_boot));
    options->parameter_overrides(parameters);
  }
  auto bridge = flapping_uav_gcs::make_mavlink_bridge_node(bo);
  auto cp = co.parameter_overrides();
  cp.push_back(rclcpp::Parameter("require_start_here", true));
  cp.push_back(rclcpp::Parameter("publish_body_setpoint", true));
  cp.push_back(rclcpp::Parameter("continuous_hold", true));
  cp.push_back(rclcpp::Parameter("invalid_velocity_as_zero", true));
  cp.push_back(rclcpp::Parameter("heartbeat_required", false));
  co.parameter_overrides(cp);
  auto control = flapping_uav_gcs::make_position_controller_node(co);
  auto probe = std::make_shared<rclcpp::Node>("position_graph_probe");
  auto pose = probe->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position", 10);
  auto uwb = probe->create_publisher<flapping_uav_msgs::msg::UwbStatus>("/uwb/status", 10);
  auto goal = probe->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  auto enable = probe->create_client<std_srvs::srv::SetBool>("/navigation/enable");
  auto cancel = probe->create_client<std_srvs::srv::Trigger>("/navigation/cancel_goal");
  flapping_uav_msgs::msg::NavigationStatus last;
  flapping_uav_msgs::msg::BodyMotionState motion;
  unsigned motion_count = 0;
  auto ss = probe->create_subscription<flapping_uav_msgs::msg::NavigationStatus>("/navigation/status", 10,
    [&](flapping_uav_msgs::msg::NavigationStatus::ConstSharedPtr m) {last = *m;});
  auto ms = probe->create_subscription<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion", 10,
    [&](flapping_uav_msgs::msg::BodyMotionState::ConstSharedPtr m) {motion = *m; ++motion_count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto node : {bridge, control, probe}) {executor.add_node(node);}
  std::vector<mavlink_set_position_target_local_ned_t> sent;
  mavlink_message_t buffer{}; mavlink_status_t parser{};
  auto step = [&]() {
      executor.spin_some(); std::array<uint8_t, 4096> bytes; ssize_t length;
      while ((length = ::recv(board.fd, bytes.data(), bytes.size(), 0)) >= 0) {
        for (ssize_t i = 0; i < length; ++i) {
          mavlink_message_t m{}; mavlink_status_t s{};
          if (mavlink_frame_char_buffer(&buffer, &parser, bytes[i], &m, &s) != MAVLINK_FRAMING_OK) {continue;}
          ASSERT_EQ(m.msgid, 84U);
          mavlink_set_position_target_local_ned_t v; mavlink_msg_set_position_target_local_ned_decode(&m, &v);
          EXPECT_EQ(v.coordinate_frame, 8); EXPECT_EQ(v.type_mask, 3559);
          EXPECT_FLOAT_EQ(v.vz, 0); EXPECT_TRUE(std::isfinite(v.vx)); sent.push_back(v);
        }
      }
      std::this_thread::sleep_for(2ms);
    };
  bool fp = true, fm = true, fa = true, fh = false, freeze = false, bad = false, valid_uwb = true;
  double actual_x = 2, actual_y = 3, actual_v = 0;
  bool turn = false;
  float height = 1.2F;
  uint32_t tm = 100, mt = 100;
  const char name[10] = "motion"; const float offset[4] = {0, 0, 0, 0};
  auto run = [&](double seconds) {
      auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      auto next = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() < end) {
        if (std::chrono::steady_clock::now() >= next) {
          tm += 20; if (!freeze) {mt += 20;}
          mavlink_message_t m;
          if (fh) {mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE); board.send(m, rx);}
          if (fa) {
            const float h = std::sqrt(.5F), cp = std::cos(std::acos(-1.F)/12), sp = std::sin(std::acos(-1.F)/12);
            // Yaw90 and pitch30: planar setpoints must NOT shrink by cos(pitch).
            mavlink_msg_attitude_quaternion_pack(1, 1, &m, tm, turn ? 0.F : h*cp, turn ? -sp : -h*sp, turn ? 0.F : h*sp, turn ? cp : h*cp, 0, 0, 0, offset); board.send(m, rx);
          }
          if (fm) {mavlink_msg_debug_vect_pack(1, 1, &m, name, static_cast<uint64_t>(mt)*1000, bad ? NAN : actual_v, 0, height); board.send(m, rx);}
          if (fp) {
            geometry_msgs::msg::PoseWithCovarianceStamped p; p.header.stamp = probe->now(); p.header.frame_id = "map";
            p.pose.pose.position.x = actual_x; p.pose.pose.position.y = actual_y; p.pose.pose.position.z = 999; pose->publish(p);
          }
          flapping_uav_msgs::msg::UwbStatus s; s.header.stamp = probe->now(); s.header.frame_id = "map";
          s.position_valid = valid_uwb; s.coordinates_confirmed = true; uwb->publish(s); next += 20ms;
        }
        step();
      }
    };
  auto start = probe->create_client<std_srvs::srv::Trigger>("/navigation/start_here");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto missing = start->async_send_request(request);
  for (int i=0; i<100; ++i) {step();}
  ASSERT_EQ(missing.wait_for(0s), std::future_status::ready);
  EXPECT_FALSE(missing.get()->success);
  fm = false; bad = true; height = NAN;
  run(.6); ASSERT_FALSE(last.enabled); ASSERT_FALSE(last.inputs_valid);
  auto started = start->async_send_request(request); run(.25);
  ASSERT_EQ(started.wait_for(0s), std::future_status::ready);
  ASSERT_TRUE(started.get()->success) << last.reason;
  EXPECT_TRUE(last.enabled); EXPECT_DOUBLE_EQ(last.goal.x, 2); EXPECT_DOUBLE_EQ(last.goal.y, 3);
  EXPECT_EQ(last.state, "WAITING_FEEDBACK"); EXPECT_TRUE(last.goal_active);
  EXPECT_FLOAT_EQ(sent.back().vx, 0);
  // A fresh NaN optical-flow packet is usable as assumed zero on the ground.
  fm = true; run(.3);
  ASSERT_TRUE(last.inputs_valid); ASSERT_TRUE(last.feedback_assumed_zero);
  EXPECT_TRUE(std::isinf(last.heartbeat_age_ms));
  EXPECT_NE(last.reason.find("heartbeat stale (diagnostic)"), std::string::npos);
  EXPECT_TRUE(std::isnan(last.actual_speed_mps));
  ASSERT_TRUE(last.reached); EXPECT_NEAR(sent.back().vx, 0, 1e-6);
  actual_x = 2.4; run(.5);
  EXPECT_FALSE(last.reached); EXPECT_DOUBLE_EQ(last.goal.x, 2);
  EXPECT_LT(sent.back().vx, -.1); EXPECT_NEAR(sent.back().vy, 0, 1e-5);
  EXPECT_FALSE(motion.velocity_valid); EXPECT_TRUE(last.feedback_assumed_zero);
  // Fallback never masks packet loss once feedback has been used.
  fm = false; run(.3);
  EXPECT_EQ(last.state, "FAULT"); EXPECT_FALSE(last.goal_active);
  EXPECT_FLOAT_EQ(sent.back().vx, 0);
  fm = true; run(.1);
  auto recapture = start->async_send_request(request); run(.15);
  ASSERT_EQ(recapture.wait_for(0s), std::future_status::ready); ASSERT_TRUE(recapture.get()->success);
  EXPECT_TRUE(last.feedback_assumed_zero);
  // Finite feedback automatically replaces the assumption, including damping.
  bad = false; height = 1.2F; actual_v = .2; run(.1);
  EXPECT_FALSE(last.feedback_assumed_zero); EXPECT_NEAR(last.actual_speed_mps, .2, 1e-6);
  actual_v = 0; actual_x += .4; run(.5);
  EXPECT_LT(sent.back().vx, -.1);
  // A delayed old UDP attitude/motion packet must not erase captured +X.
  mavlink_message_t delayed;
  const float half = std::sqrt(.5F);
  mavlink_msg_attitude_quaternion_pack(1, 1, &delayed, tm-300,
    half, 0, 0, half, 0, 0, 0, offset); board.send(delayed, rx);
  mavlink_msg_debug_vect_pack(1, 1, &delayed, name,
    static_cast<uint64_t>(mt-300)*1000, 0, 0, 1); board.send(delayed, rx);
  run(.1);
  EXPECT_TRUE(last.goal_active); EXPECT_TRUE(last.inputs_valid);
  EXPECT_LT(sent.back().vx, -.1);
  // After yaw turns 90 degrees, the same map correction becomes body right.
  turn = true; run(.2);
  EXPECT_NEAR(sent.back().vx, 0, 1e-5); EXPECT_GT(sent.back().vy, .1);
  actual_x = 2.4; actual_y = 3.4; turn = false; run(.85);
  EXPECT_NEAR(sent.back().vx, 0, 1e-5); EXPECT_GT(sent.back().vy, .1);
  fa = false; run(.3); EXPECT_EQ(last.state, "FAULT"); EXPECT_FALSE(last.goal_active);
  EXPECT_FLOAT_EQ(sent.back().vx, 0); EXPECT_FLOAT_EQ(sent.back().vy, 0);
  auto stale = start->async_send_request(request); run(.1);
  ASSERT_EQ(stale.wait_for(0s), std::future_status::ready); EXPECT_FALSE(stale.get()->success);
  fa = true; run(.2);
  auto restart = start->async_send_request(request); run(.2);
  ASSERT_EQ(restart.wait_for(0s), std::future_status::ready); ASSERT_TRUE(restart.get()->success);
  EXPECT_DOUBLE_EQ(last.goal.y, 3.4);
  actual_x += .4; run(.5); ASSERT_LT(sent.back().vx, -.1);
  tm=0; mt=0; run(.3);
  EXPECT_EQ(last.state, "FAULT"); EXPECT_FALSE(last.goal_active);
  EXPECT_FLOAT_EQ(sent.back().vx, 0); EXPECT_FLOAT_EQ(sent.back().vy, 0);
  auto enreq = std::make_shared<std_srvs::srv::SetBool::Request>(); enreq->data = true;
  auto denied = enable->async_send_request(enreq); run(.1);
  ASSERT_EQ(denied.wait_for(0s), std::future_status::ready); EXPECT_FALSE(denied.get()->success);
  auto newboot = start->async_send_request(request); run(.2);
  ASSERT_EQ(newboot.wait_for(0s), std::future_status::ready); EXPECT_TRUE(newboot.get()->success);
  for (auto node : {probe, control, bridge}) {executor.remove_node(node);}
}

TEST_F(PositionGraph, ShortInputFaultPausesAndRecoversButDeadlineAndReferenceResetLatch)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("require_start_here",true),
    rclcpp::Parameter("publish_body_setpoint",true), rclcpp::Parameter("continuous_hold",true),
    rclcpp::Parameter("attitude_convention_confirmed",true),
    rclcpp::Parameter("feedback_convention_confirmed",true),
    rclcpp::Parameter("feedback_velocity_mode","heading_xy"),
    rclcpp::Parameter("heartbeat_required",false),
    rclcpp::Parameter("input_recovery_grace_ms",250),
    rclcpp::Parameter("kp_xy",.3), rclcpp::Parameter("kd_xy",0.),
    rclcpp::Parameter("max_speed_mps",.15), rclcpp::Parameter("max_acceleration_mps2",.15),
    rclcpp::Parameter("arrival_position_tolerance_m",.05),
    rclcpp::Parameter("arrival_speed_tolerance_mps",.05), rclcpp::Parameter("arrival_dwell_s",.2),
    rclcpp::Parameter("position_timeout_ms",150), rclcpp::Parameter("attitude_timeout_ms",100),
    rclcpp::Parameter("feedback_timeout_ms",100)});
  auto control=flapping_uav_gcs::make_position_controller_node(options);
  auto probe=std::make_shared<rclcpp::Node>("input_recovery_probe");
  auto pose=probe->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position",10);
  auto uwb=probe->create_publisher<flapping_uav_msgs::msg::UwbStatus>("/uwb/status",10);
  auto attitude=probe->create_publisher<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude",10);
  auto motion=probe->create_publisher<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion",10);
  auto start=probe->create_client<std_srvs::srv::Trigger>("/navigation/start_here");
  flapping_uav_msgs::msg::NavigationStatus last;
  flapping_uav_msgs::msg::BodyVelocitySetpoint body;
  std::vector<double> valid_speeds;
  auto ns=probe->create_subscription<flapping_uav_msgs::msg::NavigationStatus>("/navigation/status",10,
    [&](flapping_uav_msgs::msg::NavigationStatus::ConstSharedPtr m){last=*m;});
  auto bs=probe->create_subscription<flapping_uav_msgs::msg::BodyVelocitySetpoint>("/fc/velocity_setpoint_body",10,
    [&](flapping_uav_msgs::msg::BodyVelocitySetpoint::ConstSharedPtr m){
      body=*m; if(m->valid){valid_speeds.push_back(std::hypot(m->velocity.x,m->velocity.y));}});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(control);executor.add_node(probe);
  bool good=true, feed_att=true; uint8_t reset=0; double x=0;
  const auto origin=std::chrono::steady_clock::now();
  auto run=[&](double seconds){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    auto next=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()<end){
      if(std::chrono::steady_clock::now()>=next){
        const auto stamp=probe->now();
        const uint64_t sample=10000000+std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-origin).count();
        geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.stamp=stamp;p.header.frame_id="map";
        p.pose.pose.position.x=x;pose->publish(p);
        flapping_uav_msgs::msg::UwbStatus u;u.header=p.header;u.position_valid=true;u.coordinates_confirmed=true;uwb->publish(u);
        if(feed_att){
          flapping_uav_msgs::msg::AttitudeState a;a.header.stamp=stamp;a.header.frame_id="local_ned";
          a.body_frame_id="base_link_frd";a.system_id=a.component_id=1;a.sample_time_us=sample;
          a.reset_counter=reset;a.valid=good;a.orientation.w=1.;a.reason=good?"valid":"fixture invalid quaternion";
          attitude->publish(a);
        }
        flapping_uav_msgs::msg::BodyMotionState m;m.header.stamp=stamp;m.header.frame_id="body_xy";
        m.system_id=m.component_id=1;m.sample_time_us=sample;m.velocity_valid=true;motion->publish(m);
        next+=10ms;
      }
      executor.spin_some();std::this_thread::sleep_for(2ms);
    }
  };
  auto begin=[&](){
    auto future=start->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    run(.1);EXPECT_EQ(future.wait_for(0s),std::future_status::ready);
    return future.get()->success;
  };
  run(.5);ASSERT_TRUE(begin());x=1.;run(.4);ASSERT_TRUE(body.valid);ASSERT_GT(std::abs(body.velocity.x),.03);
  good=false;run(.1);
  EXPECT_EQ(last.state,"INPUT_HOLD");EXPECT_TRUE(last.goal_active);EXPECT_FALSE(last.inputs_valid);
  EXPECT_FALSE(body.valid);EXPECT_FLOAT_EQ(body.velocity.x,0);
  EXPECT_NE(last.reason.find("fixture invalid quaternion"),std::string::npos);
  EXPECT_NE(last.reason.find("q(wxyz)"),std::string::npos);
  valid_speeds.clear();good=true;run(.08);
  ASSERT_TRUE(body.valid);EXPECT_EQ(last.state,"ACTIVE");EXPECT_DOUBLE_EQ(last.goal.x,0);
  ASSERT_FALSE(valid_speeds.empty());EXPECT_LE(valid_speeds.front(),.007);
  // Missing attitude follows the same bounded pause path, with a precise cause.
  feed_att=false;run(.15);EXPECT_EQ(last.state,"INPUT_HOLD");EXPECT_TRUE(last.goal_active);
  EXPECT_NE(last.reason.find("sample_age_ms="),std::string::npos);
  feed_att=true;run(.08);EXPECT_TRUE(body.valid);EXPECT_TRUE(last.goal_active);
  // Expired recovery cannot resume automatically after data becomes valid.
  good=false;run(.4);EXPECT_EQ(last.state,"FAULT");EXPECT_FALSE(last.goal_active);EXPECT_FALSE(body.valid);
  good=true;run(.1);EXPECT_TRUE(last.inputs_valid);EXPECT_FALSE(last.goal_active);EXPECT_FALSE(body.valid);
  EXPECT_NE(last.reason.find("recovery window expired"),std::string::npos);
  EXPECT_NE(last.reason.find("Inputs recovered"),std::string::npos);
  ASSERT_TRUE(begin());x=2.;run(.2);ASSERT_TRUE(body.valid);
  // A device reference change bypasses grace, including while already paused.
  good=false;run(.06);ASSERT_EQ(last.state,"INPUT_HOLD");
  good=true;++reset;run(.06);EXPECT_EQ(last.state,"FAULT");EXPECT_FALSE(last.goal_active);
  EXPECT_FALSE(body.valid);EXPECT_NE(last.reason.find("reference reset"),std::string::npos);
  executor.remove_node(probe);executor.remove_node(control);
}
