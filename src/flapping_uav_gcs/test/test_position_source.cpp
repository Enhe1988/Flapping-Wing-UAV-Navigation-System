#include <gtest/gtest.h>
#include "flapping_uav_gcs/position_source.hpp"
using flapping_uav_gcs::PositionSource;
TEST(PositionSource, HealthyModeUsesUwbAndFallbackIgnoresBadRawData)
{
  PositionSource s(.5);
  auto p=s.update(0,true,{{1,2}},true,{{.9,1.9}});ASSERT_TRUE(p.valid);EXPECT_EQ(p.source,"UWB");
  EXPECT_DOUBLE_EQ(p.position[0],1);
  p=s.update(.02,false,{{99,99}},true,{{.92,1.92}});
  ASSERT_TRUE(p.valid);EXPECT_EQ(p.source,"FLOW_ODOMETRY");EXPECT_NEAR(p.position[0],1,1e-12);
  p=s.update(.12,false,{{-99,-99}},true,{{1.02,1.82}});
  EXPECT_NEAR(p.position[0],1.1,1e-12);EXPECT_NEAR(p.position[1],1.9,1e-12);
}
TEST(PositionSource, RecoveryBlendsToRawUwbWithoutInstantPositionJump)
{
  PositionSource s(.5);s.update(0,true,{{1,2}},true,{{1,2}});
  s.update(.1,false,{{8,8}},true,{{1,2}});
  auto p=s.update(.2,true,{{1.3,2.2}},true,{{1,2}});
  EXPECT_EQ(p.source,"UWB_BLEND");EXPECT_NEAR(p.position[0],1,1e-12);
  p=s.update(.45,true,{{1.3,2.2}},true,{{1.3,2.2}});EXPECT_NEAR(p.position[0],1.15,1e-12);
  p=s.update(.71,true,{{1.3,2.2}},true,{{1.3,2.2}});EXPECT_EQ(p.source,"UWB");EXPECT_NEAR(p.position[0],1.3,1e-12);
}
TEST(PositionSource, BothInvalidNeverProduceUsableCoordinates)
{
  PositionSource s(.5);s.update(0,true,{{1,2}},false,{{NAN,NAN}});
  auto p=s.update(.1,false,{{5,5}},false,{{1,2}});EXPECT_FALSE(p.valid);EXPECT_EQ(p.source,"INVALID");
  EXPECT_NEAR(p.position[0],1,1e-12);s.reset();EXPECT_FALSE(s.update(.2,false,{{NAN,NAN}},false,{{NAN,NAN}}).valid);
}
TEST(PositionSource, InterruptedRecoveryReturnsContinuouslyToFlow)
{
  PositionSource s(.5);s.update(0,true,{{1,2}},true,{{1,2}});
  s.update(.1,false,{{9,9}},true,{{1,2}});s.update(.2,true,{{1.4,2}},true,{{1.4,2}});
  const auto p=s.update(.4,true,{{1.4,2}},true,{{1.4,2}});
  const auto q=s.update(.41,false,{{9,9}},true,{{1.41,2}});
  EXPECT_TRUE(q.valid);EXPECT_EQ(q.source,"FLOW_ODOMETRY");EXPECT_NEAR(q.position[0],p.position[0],1e-12);
}
