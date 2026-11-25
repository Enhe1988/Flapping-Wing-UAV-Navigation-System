#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/fusion_status.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
using namespace std::chrono_literals;  // NOLINT
namespace {
class FusionGraph:public ::testing::Test
{
  void SetUp()override{rclcpp::init(0,nullptr);}
  void TearDown()override{if(rclcpp::ok()){rclcpp::shutdown();}}
};
}
TEST_F(FusionGraph, ObserverAnchorsRejectsJumpsCoastsExpiresAndNeverPublishesControl)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("uwb_timeout_ms",100),rclcpp::Parameter("flow_timeout_ms",100),
    rclcpp::Parameter("attitude_timeout_ms",100),rclcpp::Parameter("max_attitude_pair_skew_ms",50),
    rclcpp::Parameter("max_flow_device_gap_ms",100),rclcpp::Parameter("max_dead_reckoning_s",.5)});
  auto fusion=flapping_uav_gcs::make_fusion_odometry_node(options);
  auto probe=std::make_shared<rclcpp::Node>("fusion_graph_probe");
  auto pose=probe->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position",10);
  auto uwb=probe->create_publisher<flapping_uav_msgs::msg::UwbStatus>("/uwb/status",10);
  auto att=probe->create_publisher<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude",10);
  auto motion=probe->create_publisher<flapping_uav_msgs::msg::BodyMotionState>("/fc/body_motion",10);
  auto align=probe->create_client<std_srvs::srv::Trigger>("/fusion/align_positive_x");
  auto reset=probe->create_client<std_srvs::srv::Trigger>("/fusion/reset");
  flapping_uav_msgs::msg::FusionStatus last;nav_msgs::msg::Odometry odom;unsigned count=0;
  auto ss=probe->create_subscription<flapping_uav_msgs::msg::FusionStatus>("/fusion/status",10,
    [&](flapping_uav_msgs::msg::FusionStatus::ConstSharedPtr m){last=*m;});
  auto os=probe->create_subscription<nav_msgs::msg::Odometry>("/fusion/odometry",10,
    [&](nav_msgs::msg::Odometry::ConstSharedPtr m){odom=*m;++count;});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(fusion);executor.add_node(probe);
  bool feed_uwb=true,valid_flow=false,freeze_flow=false;double x=1,y=-2;uint64_t sample=1000000,motion_sample=1000000;uint8_t reference=0;
  auto run=[&](double seconds){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    auto next=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()<end){
      if(std::chrono::steady_clock::now()>=next){
        sample+=20000;if(!freeze_flow){motion_sample=sample;}
        if(feed_uwb){
          geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.stamp=probe->now();p.header.frame_id="map";
          p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.position.z=999.;pose->publish(p);
          flapping_uav_msgs::msg::UwbStatus u;u.header=p.header;u.position_valid=true;u.coordinates_confirmed=true;uwb->publish(u);
        }
        flapping_uav_msgs::msg::AttitudeState a;a.header.stamp=probe->now();a.header.frame_id="local_ned";
        a.body_frame_id="base_link_frd";a.system_id=a.component_id=1;a.sample_time_us=sample;
        a.reset_counter=reference;a.orientation.w=1.;a.valid=true;att->publish(a);
        flapping_uav_msgs::msg::BodyMotionState m;m.header.stamp=probe->now();m.header.frame_id="body_xy";
        m.system_id=m.component_id=1;m.sample_time_us=motion_sample;m.velocity_valid=valid_flow;
        m.forward_velocity_mps=valid_flow?1.:NAN;m.right_velocity_mps=valid_flow?.2:NAN;m.height_up_m=999.;motion->publish(m);
        next+=20ms;
      }
      executor.spin_some();std::this_thread::sleep_for(2ms);
    }
  };
  auto call=[&](rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client){
    auto future=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());run(.12);
    EXPECT_EQ(future.wait_for(0s),std::future_status::ready);return future.get()->success;
  };
  run(.4);ASSERT_TRUE(last.valid);EXPECT_EQ(last.state,"UWB_ONLY");EXPECT_FALSE(last.flow_valid);
  EXPECT_NEAR(last.position.x,1,1e-9);EXPECT_NEAR(last.position.y,-2,1e-9);
  ASSERT_TRUE(call(align));valid_flow=true;run(.15);ASSERT_TRUE(last.alignment_valid);ASSERT_TRUE(last.flow_valid);EXPECT_EQ(last.state,"FUSED");
  EXPECT_EQ(odom.child_frame_id,"fusion_map_axes");EXPECT_DOUBLE_EQ(odom.pose.pose.position.z,0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.orientation.w,1.);EXPECT_GE(odom.pose.covariance[14],1e6);
  const double start_x=last.position.x,start_y=last.position.y;
  feed_uwb=false;run(.22);ASSERT_TRUE(last.valid);EXPECT_EQ(last.state,"DEAD_RECKONING");
  EXPECT_GT(last.position.x,start_x+.15);EXPECT_LT(last.position.y,start_y-.02);
  const auto accepted=last.accepted_uwb;
  x=6;feed_uwb=true;run(.18);EXPECT_GT(last.rejected_uwb,0U);EXPECT_EQ(last.accepted_uwb,accepted);EXPECT_LT(last.position.x,2.);
  run(.18);ASSERT_FALSE(last.valid);EXPECT_EQ(last.state,"INVALID");
  const auto expired_count=count;run(.08);EXPECT_EQ(count,expired_count);
  x=last.position.x;y=last.position.y;run(.15);ASSERT_TRUE(last.valid);EXPECT_TRUE(last.uwb_recent);
  // Invalid flow is harmless while UWB is available, but cannot bridge its loss.
  valid_flow=false;run(.12);ASSERT_TRUE(last.valid);EXPECT_EQ(last.state,"UWB_ONLY");
  feed_uwb=false;run(.15);EXPECT_FALSE(last.valid);EXPECT_FALSE(last.flow_valid);
  // Explicit observer reset and re-initialization do not touch control topics.
  ASSERT_TRUE(call(reset));EXPECT_FALSE(last.initialized);
  feed_uwb=true;valid_flow=true;run(.2);ASSERT_TRUE(last.valid);
  ++reference;run(.08);EXPECT_FALSE(last.alignment_valid);EXPECT_EQ(last.state,"UWB_ONLY");
  feed_uwb=false;run(.15);EXPECT_FALSE(last.valid);
  ASSERT_TRUE(call(align));feed_uwb=true;run(.15);ASSERT_TRUE(last.valid);
  freeze_flow=true;feed_uwb=false;run(.18);EXPECT_FALSE(last.valid);EXPECT_GT(last.duplicate_flow,0U);
  for(const auto & topic:probe->get_topic_names_and_types()){
    if(topic.first.rfind("/fusion/",0)!=0 && topic.first!="/localization/position" && topic.first!="/uwb/position_filtered" && topic.first!="/rosout" && topic.first!="/parameter_events"){
      for(const auto & publisher:probe->get_publishers_info_by_topic(topic.first)){
        EXPECT_NE(publisher.node_name(),"fusion_odometry_node")<<topic.first;
      }
    }
  }
  EXPECT_EQ(probe->count_publishers("/fc/velocity_setpoint_body"),0U);
  EXPECT_EQ(probe->count_publishers("/navigation/goal"),0U);
  executor.remove_node(probe);executor.remove_node(fusion);
}
