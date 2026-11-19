#include <algorithm>
#include <chrono>
#include <thread>
#include <set>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/bench_nodes.hpp"
#include "flapping_uav_gcs/scene_geometry.hpp"
#include "flapping_uav_gcs/alignment_session.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/heading_alignment.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace flapping_uav_gcs;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

TEST(SceneGeometry, BodyNedToMapPreservesFullTiltAndWorldAlignment)
{
  QuaternionWxyz q; const double h=std::sqrt(.5);
  ASSERT_TRUE(body_attitude_to_map({{1,0,0,0}},true,0,q));
  auto f=rotate_vector(q,{{1,0,0}}),r=rotate_vector(q,{{0,1,0}}),d=rotate_vector(q,{{0,0,1}});
  EXPECT_NEAR(f[0],0,1e-12);EXPECT_NEAR(f[1],1,1e-12);
  EXPECT_NEAR(r[0],1,1e-12);EXPECT_NEAR(d[2],-1,1e-12);
  ASSERT_TRUE(body_attitude_to_map({{h,0,0,h}},true,0,q));
  f=rotate_vector(q,{{1,0,0}});EXPECT_NEAR(f[0],1,1e-12);EXPECT_NEAR(f[1],0,1e-12);
  ASSERT_TRUE(body_attitude_to_map({{h,0,0,-h}},false,0,q));
  f=rotate_vector(q,{{1,0,0}});EXPECT_NEAR(f[0],1,1e-12);
  ASSERT_TRUE(body_attitude_to_map({{h,h,0,0}},true,0,q));
  r=rotate_vector(q,{{0,1,0}});EXPECT_NEAR(r[2],-1,1e-12);
  ASSERT_TRUE(body_attitude_to_map({{1,0,0,0}},true,std::acos(-1)/2,q));
  f=rotate_vector(q,{{1,0,0}});EXPECT_NEAR(f[0],1,1e-12);
  EXPECT_FALSE(body_attitude_to_map({{0,0,0,0}},true,0,q));
  auto p=uwb_point_to_map({{1,0,2}},std::acos(-1)/2,{{3,4,5}});
  EXPECT_NEAR(p[0],3,1e-12);EXPECT_NEAR(p[1],5,1e-12);EXPECT_NEAR(p[2],7,1e-12);
}

namespace
{
class SceneGraph : public ::testing::Test
{
  void SetUp() override {rclcpp::init(0,nullptr);}
  void TearDown() override {if(rclcpp::ok()){rclcpp::shutdown();}}
};
}

TEST_F(SceneGraph, ConfiguredAnchorMarkersStaticTfFullAttitudeAndStaleDeletion)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("anchor_coordinates_configured",true),
    rclcpp::Parameter("uwb_y_sign",-1.0),
    rclcpp::Parameter("uwb_frame_id","uwb_rh"),
    rclcpp::Parameter("anchor_z_configured",true),
    rclcpp::Parameter("anchor_positions_m",std::vector<double>{0,0,1,4,0,1,4,3,1,0,3,1}),
    rclcpp::Parameter("map_to_reference_confirmed",true),
    rclcpp::Parameter("attitude_convention_confirmed",true),
    rclcpp::Parameter("data_timeout_ms",150),
    rclcpp::Parameter("uwb_to_map_yaw_rad",std::acos(-1)/2),
    rclcpp::Parameter("uwb_to_map_translation_m",std::vector<double>{1,2,3})});
  timespec reference{};ASSERT_EQ(::clock_gettime(CLOCK_MONOTONIC,&reference),0);
  const int64_t reference_ns=static_cast<int64_t>(reference.tv_sec)*1000000000LL+reference.tv_nsec;
  std::string boot_id;{std::ifstream f("/proc/sys/kernel/random/boot_id");std::getline(f,boot_id);}
  auto parameters=options.parameter_overrides();
  parameters.push_back(rclcpp::Parameter("alignment_requires_session",true));
  parameters.push_back(rclcpp::Parameter("alignment_device_time_us",int64_t(500000000)));
  parameters.push_back(rclcpp::Parameter("alignment_host_monotonic_ns",reference_ns));
  parameters.push_back(rclcpp::Parameter("alignment_host_boot_id",boot_id));
  options.parameter_overrides(parameters);
  auto scene=make_scene_visualizer_node(options);
  auto probe=std::make_shared<rclcpp::Node>("scene_test_probe");
  auto pose=probe->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position",10);
  auto uwb=probe->create_publisher<flapping_uav_msgs::msg::UwbStatus>("/uwb/status",10);
  auto attitude=probe->create_publisher<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude",10);
  visualization_msgs::msg::MarkerArray last;tf2_msgs::msg::TFMessage statics,dynamics;
  std::set<int64_t> model_stamps,transform_stamps;
  auto markers=probe->create_subscription<visualization_msgs::msg::MarkerArray>("/debug/scene_markers",rclcpp::QoS(1).transient_local(),
    [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr m){
      last=*m;
      for(const auto & marker:m->markers){
        if(marker.id==1000&&marker.action==marker.ADD){
          model_stamps.insert(rclcpp::Time(marker.header.stamp).nanoseconds());
        }
      }
    });
  auto st=probe->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(1).transient_local(),
    [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m){statics=*m;});
  auto dt=probe->create_subscription<tf2_msgs::msg::TFMessage>("/tf",10,
    [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m){
      dynamics=*m;
      for(const auto & t:m->transforms){
        if(t.child_frame_id=="aircraft_body_frd"){
          transform_stamps.insert(rclcpp::Time(t.header.stamp).nanoseconds());
        }
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(scene);executor.add_node(probe);
  bool reset_fc=false;
  auto run=[&](double seconds,bool feed){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    auto next=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()<end){
      if(feed&&std::chrono::steady_clock::now()>=next){
        geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.stamp=probe->now();p.header.frame_id="map";
        p.pose.pose.position.x=2;p.pose.pose.position.y=3;p.pose.pose.position.z=-.5;pose->publish(p);
        flapping_uav_msgs::msg::UwbStatus s;s.header=p.header;s.position_valid=true;s.coordinates_confirmed=true;uwb->publish(s);
        flapping_uav_msgs::msg::AttitudeState a;a.header.stamp=probe->now();a.header.frame_id="local_ned";
        a.body_frame_id="base_link_frd";a.valid=true;
        timespec current{};::clock_gettime(CLOCK_MONOTONIC,&current);
        const int64_t current_ns=static_cast<int64_t>(current.tv_sec)*1000000000LL+current.tv_nsec;
        a.sample_time_us=(reset_fc?2000000:500000000)+(current_ns-reference_ns)/1000;
        const double h=std::sqrt(.5);a.orientation.w=h;a.orientation.x=h;attitude->publish(a);
        next+=20ms;
      }
      executor.spin_some();std::this_thread::sleep_for(2ms);
    }
  };
  run(.8,true);
  auto find=[&](int id){return std::find_if(last.markers.begin(),last.markers.end(),[&](const auto & m){return m.id==id&&m.action==m.ADD;});};
  for(int id:{100,102,104,106}){auto m=find(id);ASSERT_NE(m,last.markers.end());EXPECT_EQ(m->header.frame_id,"uwb_rh");}
  EXPECT_DOUBLE_EQ(find(104)->pose.position.y,-3);
  ASSERT_NE(find(40),last.markers.end());EXPECT_DOUBLE_EQ(find(40)->points[1].y,-1.3);
  ASSERT_NE(find(1000),last.markers.end());EXPECT_EQ(find(1000)->header.frame_id,"aircraft_body_frd");
  ASSERT_FALSE(statics.transforms.empty());EXPECT_EQ(statics.transforms[0].child_frame_id,"uwb_rh");
  EXPECT_DOUBLE_EQ(statics.transforms[0].transform.translation.y,2);
  ASSERT_FALSE(dynamics.transforms.empty());EXPECT_EQ(dynamics.transforms[0].child_frame_id,"aircraft_body_frd");
  unsigned matched=0;
  for(auto stamp:model_stamps){if(transform_stamps.count(stamp)){++matched;}}
  EXPECT_GE(matched,5U) << "Model markers and TF must share exact timestamps for RViz";
  EXPECT_DOUBLE_EQ(dynamics.transforms[0].transform.translation.x,2);
  EXPECT_DOUBLE_EQ(dynamics.transforms[0].transform.translation.z,-.5);
  const auto & q=dynamics.transforms[0].transform.rotation;
  auto right=rotate_vector({{q.w,q.x,q.y,q.z}},{{0,1,0}});EXPECT_NEAR(right[2],-1,1e-12);
  run(.3,false);
  EXPECT_EQ(find(1000),last.markers.end());ASSERT_NE(find(100),last.markers.end());
  EXPECT_TRUE(std::any_of(last.markers.begin(),last.markers.end(),[](const auto & m){return m.action==m.DELETEALL;}));
  reset_fc=true;run(.3,true);
  ASSERT_NE(find(1000),last.markers.end());
  EXPECT_EQ(find(1000)->header.frame_id,"aircraft_attitude_preview");
  ASSERT_NE(find(1050),last.markers.end());
  EXPECT_NE(find(1050)->text.find("SESSION-EXPIRED"),std::string::npos);
  // Runtime start_here calibration restores overlay without restarting RViz.
  auto alignment = probe->create_publisher<flapping_uav_msgs::msg::HeadingAlignment>(
    "/navigation/heading_alignment", rclcpp::QoS(1).reliable().transient_local());
  timespec current{}; ASSERT_EQ(::clock_gettime(CLOCK_MONOTONIC, &current), 0);
  const int64_t current_ns = static_cast<int64_t>(current.tv_sec)*1000000000LL+current.tv_nsec;
  flapping_uav_msgs::msg::HeadingAlignment a;
  a.header.frame_id = "map"; a.header.stamp = probe->now(); a.valid = true;
  a.quaternion_body_to_ned = true; a.map_to_enu_yaw_rad = std::acos(-1)/2;
  a.device_time_us = 2000000+(current_ns-reference_ns)/1000;
  a.host_monotonic_ns = current_ns; a.host_boot_id = boot_id; a.max_clock_skew_ms = 1000;
  alignment->publish(a); run(.3, true);
  ASSERT_NE(find(1000),last.markers.end());
  EXPECT_EQ(find(1000)->header.frame_id,"aircraft_body_frd");
  const auto rotation = dynamics.transforms[0].transform.rotation;
  auto forward = rotate_vector({{rotation.w,rotation.x,rotation.y,rotation.z}},{{1,0,0}});
  EXPECT_NEAR(forward[0],1,1e-12); EXPECT_NEAR(forward[1],0,1e-12);
  a.valid = false; alignment->publish(a); run(.2, true);
  ASSERT_NE(find(1000),last.markers.end());
  EXPECT_EQ(find(1000)->header.frame_id,"aircraft_attitude_preview");
}

TEST_F(SceneGraph, MissingCoordinatesNeverCreateFakeAnchorsAndUnalignedAttitudeIsSeparate)
{
  auto scene=make_scene_visualizer_node(rclcpp::NodeOptions());
  auto probe=std::make_shared<rclcpp::Node>("unaligned_scene_probe");
  auto attitude=probe->create_publisher<flapping_uav_msgs::msg::AttitudeState>("/fc/attitude",10);
  visualization_msgs::msg::MarkerArray last;
  auto sub=probe->create_subscription<visualization_msgs::msg::MarkerArray>("/debug/scene_markers",rclcpp::QoS(1).transient_local(),
    [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr m){last=*m;});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(scene);executor.add_node(probe);
  const auto end=std::chrono::steady_clock::now()+700ms;uint64_t sample=0;
  while(std::chrono::steady_clock::now()<end){
    flapping_uav_msgs::msg::AttitudeState a;a.header.stamp=probe->now();a.header.frame_id="local_ned";
    a.body_frame_id="base_link_frd";a.valid=true;a.sample_time_us=++sample;a.orientation.w=1;
    attitude->publish(a);executor.spin_some();std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(last.markers.empty());bool preview=false;
  for(const auto & m:last.markers){
    EXPECT_FALSE(m.action==m.ADD&&m.id>=100&&m.id<=107);
    EXPECT_NE(m.header.frame_id,"aircraft_body_frd");
    if(m.id==1000){preview=true;EXPECT_EQ(m.header.frame_id,"aircraft_attitude_preview");}
  }
  EXPECT_TRUE(preview);
}
