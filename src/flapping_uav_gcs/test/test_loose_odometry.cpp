#include <gtest/gtest.h>
#include "flapping_uav_gcs/loose_odometry.hpp"
using namespace flapping_uav_gcs;  // NOLINT
namespace {
void initialize(LooseOdometry & f,double x=0,double y=0)
{
  f.set_flow(0,{{0,0}});
  EXPECT_FALSE(f.correct(0,{{x,y}}));EXPECT_FALSE(f.correct(.02,{{x,y}}));
  EXPECT_TRUE(f.correct(.04,{{x,y}}));
}
}
TEST(LooseOdometry, ConfirmsInitializationAndMatchesHealthyUwb)
{
  LooseOdometry f({});initialize(f,2,-3);
  f.set_flow(.05,{{1,.2}});auto predicted=f.estimate(.1);EXPECT_GT(predicted.position[0],2);
  ASSERT_TRUE(f.correct(.1,{{2.08,-2.99}}));auto out=f.estimate(.1);
  EXPECT_TRUE(out.valid);EXPECT_EQ(out.state,"FUSED");
  EXPECT_NEAR(out.position[0],2.08,1e-12);EXPECT_NEAR(out.position[1],-2.99,1e-12);
}
TEST(LooseOdometry, IntegratesFlowDuringOutageAndStopsAtHorizon)
{
  LooseOdometry f({});initialize(f);f.set_flow(.04,{{1,-.2}});
  for(int i=1;i<=110;++i){double t=.04+i*.02;f.set_flow(t,{{1,-.2}});}
  auto out=f.estimate(2.24);EXPECT_FALSE(out.valid);EXPECT_EQ(out.state,"INVALID");
  EXPECT_NEAR(out.position[0],2.,1e-9);EXPECT_NEAR(out.position[1],-.4,1e-9);
  EXPECT_DOUBLE_EQ(out.position_time,2.04);
}
TEST(LooseOdometry, RejectedJumpNeverMovesAnchorOrExtendsFallback)
{
  LooseOdometry f({});initialize(f);f.set_flow(.04,{{.2,0}});
  for(int i=1;i<=120;++i){double t=.04+i*.02;f.set_flow(t,{{.2,0}});EXPECT_FALSE(f.correct(t,{{8,8}}));}
  auto out=f.estimate(2.44);EXPECT_FALSE(out.valid);EXPECT_EQ(out.accepted,1U);EXPECT_EQ(out.rejected,120U);
  EXPECT_LE(out.position[0],.400000001);EXPECT_NEAR(out.position[1],0,1e-12);
}
TEST(LooseOdometry, RequiresConsecutiveRecoveryAndDoesNotPoisonGoodReference)
{
  LooseOdometry f({});initialize(f);f.set_flow(.04,{{0,0}});
  EXPECT_FALSE(f.correct(.1,{{5,0}}));
  EXPECT_FALSE(f.correct(.12,{{.01,0}}));EXPECT_FALSE(f.correct(.14,{{.02,0}}));
  EXPECT_TRUE(f.correct(.16,{{.03,0}}));auto out=f.estimate(.16);
  EXPECT_TRUE(out.valid);EXPECT_NEAR(out.position[0],.03,1e-12);
}
TEST(LooseOdometry, InvalidFlowIsNeverAZeroMeasurement)
{
  LooseOdometry f({});initialize(f,1,2);f.invalidate_flow(.05,"NaN optical flow");
  auto out=f.estimate(.1);EXPECT_TRUE(out.valid);EXPECT_EQ(out.state,"UWB_ONLY");
  f.unavailable_uwb(.11,"bad UWB");out=f.estimate(.12);
  EXPECT_FALSE(out.valid);EXPECT_EQ(out.state,"INVALID");EXPECT_NEAR(out.position[0],1,1e-12);
  f.set_flow(.13,{{1,0}});out=f.estimate(.14);EXPECT_FALSE(out.valid);
  EXPECT_FALSE(f.correct(.15,{{1,2}}));EXPECT_FALSE(f.correct(.17,{{1,2}}));
  EXPECT_TRUE(f.correct(.19,{{1,2}}));EXPECT_TRUE(f.estimate(.2).valid);
}
TEST(LooseOdometry, FlowTimeoutDoesNotIntegrateAcrossMissingInterval)
{
  LooseOdometry f({});initialize(f);f.set_flow(.04,{{1,0}});
  auto out=f.estimate(.9);EXPECT_FALSE(out.valid);EXPECT_NEAR(out.position[0],.3,1e-12);
  EXPECT_NEAR(out.position_time,.34,1e-12);
  f.set_flow(.91,{{1,0}});out=f.estimate(.93);
  EXPECT_FALSE(out.valid);EXPECT_NEAR(out.position[0],.3,1e-12);
}
TEST(LooseOdometry, CovarianceModelGrowsAndUwbCorrectionResetsIt)
{
  LooseOdometry f({});initialize(f);f.set_flow(.04,{{.2,0}});
  auto out=f.estimate(.14);EXPECT_GT(out.covariance[0],.0025);
  const std::array<double,3> measured{{.01,.002,.02}};
  ASSERT_TRUE(f.correct(.14,{{.02,0}},&measured));out=f.estimate(.14);
  EXPECT_DOUBLE_EQ(out.covariance[0],.01);EXPECT_DOUBLE_EQ(out.covariance[1],.002);
  f.reset();EXPECT_FALSE(f.estimate(.15).initialized);
}
TEST(LooseOdometry, PartialCorrectionIsAnExplicitOption)
{
  LooseOdometryConfig c;c.uwb_correction_gain=.5;LooseOdometry f(c);initialize(f);
  ASSERT_TRUE(f.correct(.05,{{.1,0}}));EXPECT_NEAR(f.estimate(.05).position[0],.05,1e-12);
  c.innovation_gate_m=NAN;EXPECT_THROW(LooseOdometry bad(c),std::invalid_argument);
}
