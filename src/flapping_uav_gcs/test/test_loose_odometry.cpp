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
TEST(LooseOdometry, ReliableUwbChecksEveryFixButSynchronizesOdometryPeriodically)
{
  LooseOdometryConfig c;c.uwb_sync_period_s=.2;LooseOdometry f(c);initialize(f);f.set_flow(.04,{{1,0}});
  ASSERT_TRUE(f.correct(.14,{{0,0}}));auto p=f.estimate(.14);
  EXPECT_NEAR(p.position[0],.1,1e-12);EXPECT_NEAR(p.reliable_uwb[0],0,1e-12);EXPECT_EQ(p.syncs,1U);
  ASSERT_TRUE(f.correct(.25,{{0,0}}));p=f.estimate(.25);EXPECT_NEAR(p.position[0],0,1e-12);EXPECT_EQ(p.syncs,2U);
}
TEST(LooseOdometry, UwbRecoveryRequiresBothTimeAndConsecutiveSamples)
{
  LooseOdometryConfig c;c.recovery_confirmation_s=.2;c.recovery_confirmation_samples=3;
  LooseOdometry f(c);initialize(f);f.set_flow(.04,{{0,0}});EXPECT_FALSE(f.correct(.06,{{5,0}}));
  EXPECT_FALSE(f.correct(.08,{{0,0}}));EXPECT_FALSE(f.correct(.10,{{0,0}}));EXPECT_FALSE(f.correct(.12,{{0,0}}));
  for(int i=7;i<=13;++i){EXPECT_FALSE(f.correct(i*.02,{{0,0}}));}
  EXPECT_TRUE(f.correct(.30,{{0,0}}));EXPECT_TRUE(f.estimate(.30).uwb_recent);
}

TEST(UwbLowPass, RealTimeConstantAttenuatesNoiseAndIsIndependentOfSampleRate)
{
  for(double dt:{.01,.02,.05}){
    UwbLowPass f(true,.1,.5);ASSERT_TRUE(f.update(0,{{0,2}}));
    for(int i=1;i<=static_cast<int>(std::round(.1/dt));++i){ASSERT_TRUE(f.update(i*dt,{{1,2}}));}
    EXPECT_NEAR(f.value()[0],1-std::exp(-1.),1e-12);EXPECT_DOUBLE_EQ(f.value()[1],2);
  }
  UwbLowPass f(true,.1,.5);f.update(0,{{0,0}});double square=0;
  for(int i=1;i<=500;++i){f.update(i*.02,{{i%2?.1:-.1,0}});if(i>50){square+=f.value()[0]*f.value()[0];}}
  EXPECT_LT(std::sqrt(square/450),.02); // Raw alternating noise RMS is 0.10 m.
}

TEST(UwbLowPass, InvalidDuplicateAndOldSamplesDoNotPoisonOrRenewState)
{
  UwbLowPass f(true,.1,.15);ASSERT_TRUE(f.update(1,{{1,2}}));
  EXPECT_FALSE(f.update(1,{{9,9}}));EXPECT_FALSE(f.update(.9,{{9,9}}));
  EXPECT_FALSE(f.update(1.01,{{NAN,9}}));EXPECT_EQ(f.value(),(std::array<double,2>{{1,2}}));
  ASSERT_TRUE(f.update(1.2,{{3,4}}));EXPECT_DOUBLE_EQ(f.value()[0],3);
  f.reset();ASSERT_TRUE(f.update(0,{{8,7}}));EXPECT_DOUBLE_EQ(f.value()[0],8);
  UwbLowPass bypass(false,.1,.5);bypass.update(0,{{0,0}});bypass.update(.01,{{1,2}});
  EXPECT_DOUBLE_EQ(bypass.value()[0],1);EXPECT_THROW(UwbLowPass invalid(true,0,.5),std::invalid_argument);
}

TEST(LooseOdometry, RawJumpGatePrecedesSmoothingAndRecoveryRestartsAtFreshFix)
{
  LooseOdometryConfig c;c.uwb_filter_enabled=true;c.uwb_filter_tau_s=.1;
  LooseOdometry f(c);initialize(f);f.set_flow(.04,{{0,0}});
  ASSERT_TRUE(f.correct(.06,{{.1,0}}));const auto before=f.estimate(.06);
  EXPECT_NEAR(before.reliable_uwb[0],.1*(1-std::exp(-.2)),1e-12);
  EXPECT_NEAR(before.position[0],before.reliable_uwb[0],1e-12);
  EXPECT_FALSE(f.correct(.08,{{5,0}}));const auto rejected=f.estimate(.08);
  EXPECT_FALSE(rejected.uwb_recent);EXPECT_EQ(rejected.reliable_uwb,before.reliable_uwb);
  EXPECT_NEAR(rejected.uwb_age,.02,1e-12); // Rejected point did not refresh age.
  EXPECT_FALSE(f.correct(.10,{{.05,0}}));EXPECT_FALSE(f.correct(.12,{{.05,0}}));
  ASSERT_TRUE(f.correct(.14,{{.05,0}}));EXPECT_NEAR(f.estimate(.14).reliable_uwb[0],.05,1e-12);
  f.reset();initialize(f,2,3);EXPECT_NEAR(f.estimate(.2).reliable_uwb[0],2,1e-12);
}

TEST(LooseOdometry, Observed240msUwbGapKeepsHealthyPositionButTrueOutageAndJumpsDoNot)
{
  LooseOdometryConfig c;c.uwb_timeout_s=.3;c.recovery_confirmation_s=.5;
  c.recovery_confirmation_samples=10;c.uwb_filter_enabled=true;
  LooseOdometry f(c);initialize(f);f.invalidate_flow(.05,"Synthetic missing flow");
  // Actual flight had a 240 ms gap with unusable optical propagation. This
  // must not force a recovery cycle while UWB remains within its valid age.
  auto before=f.estimate(.27);ASSERT_TRUE(before.valid);EXPECT_TRUE(before.uwb_recent);
  ASSERT_TRUE(f.correct(.28,{{.1,0}}));EXPECT_TRUE(f.estimate(.28).uwb_recent);
  EXPECT_FALSE(f.correct(.30,{{5,5}}));EXPECT_FALSE(f.estimate(.30).valid);
  f.reset();initialize(f);f.invalidate_flow(.05,"Synthetic missing flow");
  EXPECT_FALSE(f.estimate(.35).valid); // A real gap >300 ms still stops.
}
