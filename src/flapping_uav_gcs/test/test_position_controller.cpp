#include <limits>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/position_controller.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_gcs/position_attitude_controller.hpp"

using namespace flapping_uav_gcs;  // NOLINT

TEST(PositionAttitude, IndependentGainsAndFlightControllerAngleSigns)
{
  PositionAttitudeConfig cfg;cfg.kp_x_deg_per_m=4;cfg.kp_y_deg_per_m=8;cfg.max_rate_deg_s=1000;
  PositionAttitudeController control(cfg);const double s=std::sqrt(.5);
  const auto out=control.step({{0,0}},{{1,-.5}},0,{{s,0,0,s}},true,.1);
  ASSERT_TRUE(out.valid);EXPECT_NEAR(out.pitch_deg,4,1e-12);EXPECT_NEAR(out.roll_deg,-4,1e-12);
}

TEST(PositionAttitude, ThirtyDegreeLimitIsPerAxisAfterRotation)
{
  PositionAttitudeConfig cfg;cfg.max_rate_deg_s=10000;PositionAttitudeController control(cfg);
  const double s=std::sqrt(.5);
  const auto out=control.step({{0,0}},{{100,-100}},0,{{s,0,0,s}},true,.1);
  ASSERT_TRUE(out.valid);EXPECT_DOUBLE_EQ(out.pitch_deg,30);EXPECT_DOUBLE_EQ(out.roll_deg,-30);
  const auto rotated=control.step({{0,0}},{{1,0}},std::acos(-1.)/2,{{s,0,0,s}},true,.1);
  EXPECT_NEAR(rotated.pitch_deg,0,1e-12);EXPECT_NEAR(rotated.roll_deg,5,1e-12);
}

TEST(PositionAttitude, SlewUsesDegreesPerSecondAndResetClearsPreviousTilt)
{
  PositionAttitudeConfig cfg;cfg.max_rate_deg_s=20;PositionAttitudeController control(cfg);
  const double s=std::sqrt(.5);
  EXPECT_NEAR(control.step({{0,0}},{{10,0}},0,{{s,0,0,s}},true,.1).pitch_deg,2,1e-12);
  EXPECT_NEAR(control.step({{0,0}},{{10,0}},0,{{s,0,0,s}},true,5).pitch_deg,4,1e-12);
  control.reset();EXPECT_NEAR(control.step({{0,0}},{{10,0}},0,{{s,0,0,s}},true,.1).pitch_deg,2,1e-12);
}

TEST(PositionAttitude, PositionDwellAndMissingInputNeverRequireOpticalFlow)
{
  PositionAttitudeConfig cfg;cfg.dwell_s=.2;PositionAttitudeController control(cfg);
  EXPECT_FALSE(control.step({{1,1}},{{1,1}},0,{{1,0,0,0}},true,.1).reached);
  EXPECT_TRUE(control.step({{1,1}},{{1,1}},0,{{1,0,0,0}},true,.1).reached);
  auto bad=control.step({{NAN,1}},{{1,1}},0,{{1,0,0,0}},true,.1);
  EXPECT_FALSE(bad.valid);EXPECT_DOUBLE_EQ(bad.pitch_deg,0);EXPECT_DOUBLE_EQ(bad.roll_deg,0);
  cfg.max_angle_deg=30.01;EXPECT_FALSE(cfg.configured());
}

TEST(PositionAttitude, DerivativeUsesMeasurementTimeAndHasNoWaypointKick)
{
  PositionAttitudeConfig cfg;cfg.kd_x_deg_s_per_m=3;cfg.kd_y_deg_s_per_m=4;
  cfg.velocity_filter_tau_s=0;cfg.max_rate_deg_s=10000;
  PositionAttitudeController control(cfg);const double s=std::sqrt(.5);
  auto step=[&](XY p,XY goal,double time){return control.step(p,goal,0,{{s,0,0,s}},true,.02,false,time);};
  auto out=step({{0,0}},{{1,0}},10);EXPECT_FALSE(out.velocity_valid);
  out=step({{.1,0}},{{1,0}},10.1);
  ASSERT_TRUE(out.velocity_valid);EXPECT_NEAR(out.velocity_map_mps[0],1,1e-10);
  EXPECT_NEAR(out.d_map_deg[0],-3,1e-10);
  const double derivative=out.d_map_deg[0];
  out=step({{.1,0}},{{-.5,0}},10.1);
  EXPECT_DOUBLE_EQ(out.d_map_deg[0],derivative);  // held sample + changed goal
  out=step({{.1,0}},{{-.5,0}},10.2);EXPECT_NEAR(out.d_map_deg[0],0,1e-12);
  out=step({{2,0}},{{-.5,0}},11);EXPECT_FALSE(out.velocity_valid);
  EXPECT_DOUBLE_EQ(out.d_map_deg[0],0);  // gap re-primes instead of kicking
  EXPECT_FALSE(step({{2,0}},{{-.5,0}},10).valid);  // clock regression
}

TEST(PositionAttitude, IntegralIsBoundedRetainedAtGoalAndNotRepeatedForHeldSamples)
{
  PositionAttitudeConfig cfg;cfg.ki_x_deg_per_m_s=1;cfg.ki_y_deg_per_m_s=1;
  cfg.max_rate_deg_s=10000;cfg.integral_zone_m=2;cfg.velocity_limit_mps=100;
  PositionAttitudeController control(cfg);const double s=std::sqrt(.5);
  auto step=[&](XY pos,XY goal,double time){return control.step(pos,goal,0,{{s,0,0,s}},true,.1,false,time);};
  auto out=step({{0,0}},{{.5,.5}},1);
  for(int i=1;i<=100;++i){out=step({{0,0}},{{.5,.5}},1+i*.1);}
  EXPECT_NEAR(std::hypot(out.i_map_deg[0],out.i_map_deg[1]),2,1e-10);
  const XY integral=out.i_map_deg;
  out=step({{0,0}},{{.5,.5}},11);EXPECT_EQ(out.i_map_deg,integral);EXPECT_FALSE(out.integrating);
  // Reaching the goal keeps learned trim instead of resetting the command.
  for(int i=1;i<=30;++i){out=step({{.5,.5}},{{.5,.5}},11+i*.1);}
  ASSERT_TRUE(out.reached);EXPECT_EQ(out.i_map_deg,integral);
  EXPECT_NEAR(out.pitch_deg,integral[0],1e-10);
  EXPECT_NEAR(out.roll_deg,integral[1],1e-10);
  out=step({{.5,.5}},{{.4,.5}},14.1);
  EXPECT_LT(std::abs(out.i_map_deg[0]),.02);EXPECT_DOUBLE_EQ(out.i_map_deg[1],0);
  control.reset();out=step({{.5,.5}},{{.4,.5}},14.2);
  EXPECT_DOUBLE_EQ(out.i_map_deg[0],0);EXPECT_FALSE(out.velocity_valid);
}

TEST(PositionAttitude, IntegrationFreezesOutsideZoneDuringMotionAndAtAngleOrSlewLimits)
{
  PositionAttitudeConfig cfg;cfg.ki_x_deg_per_m_s=2;cfg.ki_y_deg_per_m_s=2;
  cfg.max_rate_deg_s=10000;cfg.kp_x_deg_per_m=100;
  const double s=std::sqrt(.5);PositionAttitudeController saturated(cfg);
  auto step=[&](PositionAttitudeController & c,XY p,XY goal,double time){
    return c.step(p,goal,0,{{s,0,0,s}},true,.1,false,time);};
  step(saturated,{{0,0}},{{.5,0}},1);
  for(int i=1;i<=100;++i){
    const auto out=step(saturated,{{0,0}},{{.5,0}},1+i*.1);
    EXPECT_DOUBLE_EQ(out.pitch_deg,30);EXPECT_DOUBLE_EQ(out.i_map_deg[0],0);
  }
  cfg.kp_x_deg_per_m=5;cfg.max_rate_deg_s=1;PositionAttitudeController slew(cfg);
  step(slew,{{0,0}},{{.5,0}},1);
  auto out=step(slew,{{0,0}},{{.5,0}},1.1);
  EXPECT_DOUBLE_EQ(out.i_map_deg[0],0);EXPECT_TRUE(out.limited);
  cfg.max_rate_deg_s=10000;PositionAttitudeController zone(cfg);
  step(zone,{{0,0}},{{2,0}},1);out=step(zone,{{0,0}},{{2,0}},1.1);
  EXPECT_DOUBLE_EQ(out.i_map_deg[0],0);
  PositionAttitudeController moving(cfg);step(moving,{{0,0}},{{.5,0}},1);
  out=step(moving,{{.2,0}},{{.5,0}},1.1);
  EXPECT_GT(out.velocity_map_mps[0],cfg.integral_max_speed_mps);
  EXPECT_DOUBLE_EQ(out.i_map_deg[0],0);
}

TEST(PositionAttitude, NoisyDerivativeIsFilteredBoundedAndExpiresWithHeldPose)
{
  PositionAttitudeConfig cfg;cfg.kd_x_deg_s_per_m=20;cfg.kd_y_deg_s_per_m=20;
  cfg.max_rate_deg_s=10000;PositionAttitudeController control(cfg);const double s=std::sqrt(.5);
  control.step({{0,0}},{{1,1}},0,{{s,0,0,s}},true,.02,false,1);
  auto out=control.step({{100,100}},{{1,1}},0,{{s,0,0,s}},true,.02,false,1.02);
  ASSERT_TRUE(out.velocity_valid);EXPECT_LT(std::hypot(out.velocity_map_mps[0],out.velocity_map_mps[1]),.3);
  EXPECT_LE(std::hypot(out.d_map_deg[0],out.d_map_deg[1]),cfg.derivative_limit_deg+1e-12);
  for(int i=0;i<20;++i){out=control.step({{100,100}},{{1,1}},0,{{s,0,0,s}},true,.02,false,1.02);}
  EXPECT_FALSE(out.velocity_valid);EXPECT_TRUE(std::isnan(out.velocity_map_mps[0]));
  EXPECT_DOUBLE_EQ(out.d_map_deg[0],0);
  cfg.ki_x_deg_per_m_s=-1;EXPECT_FALSE(cfg.configured());
  cfg.ki_x_deg_per_m_s=0;cfg.kd_y_deg_s_per_m=NAN;EXPECT_FALSE(cfg.configured());
}

TEST(PositionAttitude, PIDRemovesConstantDisturbanceInPointMassPlantWithoutOpticalFlow)
{
  const auto simulate=[](double ki){
    PositionAttitudeConfig cfg;cfg.kp_x_deg_per_m=10;cfg.kd_x_deg_s_per_m=3;
    cfg.ki_x_deg_per_m_s=ki;PositionAttitudeController control(cfg);
    double position=0,velocity=0;const double s=std::sqrt(.5);
    for(int i=0;i<20000;++i){
      const auto out=control.step({{position,0}},{{0,0}},0,{{s,0,0,s}},true,.01,false,1+i*.01);
      EXPECT_TRUE(out.valid);EXPECT_LE(std::abs(out.pitch_deg),30);
      // Illustrative plant, not a fitted flight model: angle gives horizontal
      // acceleration, with drag and a constant external disturbance.
      velocity+=(.15*out.pitch_deg-.5*velocity-.3)*.01;position+=velocity*.01;
    }
    return std::abs(position);
  };
  const double pd_error=simulate(0),pid_error=simulate(.5);
  EXPECT_GT(pd_error,.15);EXPECT_LT(pid_error,.01);EXPECT_LT(pid_error,pd_error*.1);
}

TEST(PositionController, CompactPurePAcceptsMissingSpeedWithoutReportingMeasuredZero)
{
  PositionControlConfig config{1,0,1,100,.05,.05,.2};
  config.velocity_feedback_required=false;
  PositionController controller(config);ASSERT_TRUE(controller.set_goal({{1,0}}));
  auto result=controller.step({{0,0}},{{NAN,NAN}},.1);
  ASSERT_TRUE(result.valid);EXPECT_TRUE(std::isnan(result.speed));EXPECT_NEAR(result.velocity[0],1,1e-12);
  for(int i=0;i<4;++i){result=controller.step({{1,0}},{{NAN,NAN}},.05);}
  EXPECT_TRUE(result.reached);EXPECT_TRUE(std::isnan(result.speed));
  config.kd_x=.1;EXPECT_FALSE(config.configured());
}

TEST(PositionController, VectorSpeedAndAccelerationLimits)
{
  PositionControlConfig config{1, .2, .5, .2, .05, .05, .2};
  PositionController controller(config);
  ASSERT_TRUE(controller.set_goal({{10, 10}}));
  XY previous{{0, 0}};
  for (int i = 0; i < 100; ++i) {
    auto out = controller.step({{0, 0}}, {{0, 0}}, .1);
    ASSERT_TRUE(out.valid);
    EXPECT_LE(std::hypot(out.velocity[0], out.velocity[1]), .5 + 1e-12);
    EXPECT_LE(std::hypot(out.velocity[0] - previous[0], out.velocity[1] - previous[1]), .02 + 1e-12);
    previous = out.velocity;
  }
  EXPECT_NEAR(std::hypot(previous[0], previous[1]), .5, 1e-12);
}

TEST(PositionController, IndependentMapAxisGainsApplyToFeedbackAndFeedforwardError)
{
  PositionControlConfig config{1,.1,10,100,.05,.05,.2};
  config.kp_x=2;config.kp_y=.5;config.kd_x=.3;config.kd_y=.1;
  ASSERT_TRUE(config.configured());
  PositionController controller(config);controller.set_goal({{1,-1}});
  const auto out=controller.step({{0,0}},{{.2,-.4}},.1,{{.4,.1}},true);
  ASSERT_TRUE(out.valid);EXPECT_NEAR(out.velocity[0],2.46,1e-12);EXPECT_NEAR(out.velocity[1],-.35,1e-12);
  config.kp_y=-1;EXPECT_FALSE(config.configured());
  config.kp_y=.5;config.kd_x=std::numeric_limits<double>::infinity();EXPECT_FALSE(config.configured());
}

TEST(PositionController, DampingAndArrivalRequireLowSpeedAndContinuousDwell)
{
  PositionController controller({1, .5, 1, 100, .05, .05, .2});
  controller.set_goal({{0, 0}});
  for (int i = 0; i < 10; ++i) {
    auto out = controller.step({{0, 0}}, {{.4, 0}}, .05);
    EXPECT_FALSE(out.reached); EXPECT_NEAR(out.velocity[0], -.2, 1e-12);
  }
  for (int i = 0; i < 2; ++i) {
    EXPECT_FALSE(controller.step({{0, 0}}, {{0, 0}}, .05).reached);
  }
  EXPECT_FALSE(controller.step({{.2, 0}}, {{0, 0}}, .05).reached);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(controller.step({{0, 0}}, {{0, 0}}, .05).reached);
  }
  auto reached = controller.step({{0, 0}}, {{0, 0}}, .05);
  EXPECT_TRUE(reached.reached); EXPECT_DOUBLE_EQ(reached.velocity[0], 0);
  controller.stop(); EXPECT_FALSE(controller.has_goal());
}

TEST(PositionController, VirtualPlantConvergesAndNewGoalRespectsAcceleration)
{
  PositionController controller({.8, .2, .4, .3, .05, .05, .3});
  controller.set_goal({{1.5, -.6}});
  XY position{{0, 0}}, velocity{{0, 0}};
  bool reached = false;
  for (int i = 0; i < 2000; ++i) {
    auto out = controller.step(position, velocity, .02);
    ASSERT_TRUE(out.valid);
    if (i == 20) {
      controller.set_goal({{1.5, -.6}});
      auto next = controller.step(position, velocity, .02);
      EXPECT_LE(std::hypot(next.velocity[0] - out.velocity[0], next.velocity[1] - out.velocity[1]), .006 + 1e-12);
    }
    velocity = out.velocity;
    for (size_t j = 0; j < 2; ++j) {position[j] += velocity[j] * .02;}
    if (out.reached) {reached = true; break;}
  }
  EXPECT_TRUE(reached);
  EXPECT_LT(std::hypot(position[0] - 1.5, position[1] + .6), .05);
}

TEST(PositionController, UnconfiguredAndInvalidInputsNeverProduceMotion)
{
  PositionController unconfigured({}); unconfigured.set_goal({{1, 0}});
  EXPECT_FALSE(unconfigured.step({{0, 0}}, {{0, 0}}, .02).valid);
  PositionController controller({1, 0, 1, 1, .1, .1, .2});
  EXPECT_FALSE(controller.set_goal({{NAN, 0}}));
  controller.set_goal({{1, 0}});
  auto out = controller.step({{0, 0}}, {{NAN, 0}}, .02);
  EXPECT_FALSE(out.valid); EXPECT_DOUBLE_EQ(out.velocity[0], 0);
  EXPECT_FALSE(controller.has_goal());
}

TEST(FeedbackRotation, HeadingCompensationAlignmentInverseAndMissingVertical)
{
  Velocity3 output;
  const double half = std::sqrt(.5);
  ASSERT_TRUE(feedback_xy_to_map(.4, .1, 0, {{half, 0, 0, half}}, true, true, false, output));
  EXPECT_NEAR(output[0], .4, 1e-12); EXPECT_NEAR(output[1], -.1, 1e-12);
  ASSERT_TRUE(feedback_xy_to_map(.4, .1, 0, {{half, 0, 0, -half}}, false, true, false, output));
  EXPECT_NEAR(output[0], .4, 1e-12);
  ASSERT_TRUE(feedback_xy_to_map(1, 0, std::acos(-1)/2, {{1, 0, 0, 0}}, true, true, false, output));
  EXPECT_NEAR(output[0], 1, 1e-12); EXPECT_NEAR(output[1], 0, 1e-12);
  const double angle = std::acos(-1)/12;
  QuaternionWxyz tilt{{std::cos(angle), 0, std::sin(angle), 0}};
  // Missing body vz cannot be replaced with ID250's height or invented zero.
  EXPECT_FALSE(feedback_xy_to_map(std::sqrt(.75), 0, 0, tilt, true, false, false, output));
  ASSERT_TRUE(feedback_xy_to_map(std::sqrt(.75), 0, 0, tilt, true, false, true, output));
  EXPECT_NEAR(output[0], 0, 1e-12); EXPECT_NEAR(output[1], 1, 1e-12);
  EXPECT_FALSE(feedback_xy_to_map(1, 0, 0, {{0, 0, 0, 0}}, true, true, false, output));
  const double cp = std::cos(angle), sp = std::sin(angle);
  ASSERT_TRUE(map_velocity_to_heading({{.4, 0, 0}}, 0,
    {{half*cp, -half*sp, half*sp, half*cp}}, true, output));
  EXPECT_NEAR(output[0], .4, 1e-12); EXPECT_NEAR(output[1], 0, 1e-12);
  EXPECT_DOUBLE_EQ(output[2], 0);
  EXPECT_FALSE(map_velocity_to_heading({{0, 0, 1}}, 0, {{1, 0, 0, 0}}, true, output));
}

TEST(PositionController, ContinuousHoldResumesCorrectionAfterArrival)
{
  PositionControlConfig config{.3, 0, .15, .15, .05, .05, .5};
  config.continuous_hold = true;
  PositionController controller(config);
  controller.set_goal({{2, 3}});
  for (int i = 0; i < 30; ++i) {controller.step({{2, 3}}, {{0, 0}}, .02);}
  ASSERT_TRUE(controller.step({{2, 3}}, {{0, 0}}, .02).reached);
  auto moved = controller.step({{2.4, 3}}, {{0, 0}}, .02);
  EXPECT_FALSE(moved.reached); EXPECT_LT(moved.velocity[0], 0);
  EXPECT_NEAR(moved.velocity[0], -.003, 1e-9);
  for (int i = 0; i < 100; ++i) {moved = controller.step({{2.4, 3}}, {{0, 0}}, .02);}
  EXPECT_NEAR(moved.velocity[0], -.12, 1e-9);
  EXPECT_DOUBLE_EQ(controller.goal()[0], 2);
}

TEST(PositionController, PauseRetainsGoalAndResumesAccelerationFromZero)
{
  PositionController c({.3, 0, .15, .15, .05, .05, .5});
  c.set_goal({{1, 0}});
  for (int i=0;i<100;++i) {c.step({{0, 0}}, {{0, 0}}, .02);}
  c.pause();
  ASSERT_TRUE(c.has_goal()); EXPECT_DOUBLE_EQ(c.goal()[0], 1);
  auto resumed=c.step({{0, 0}}, {{0, 0}}, .02);
  EXPECT_NEAR(resumed.velocity[0], .003, 1e-12);
}

TEST(PositionController, MovingReferenceUsesFeedforwardWithoutFalseArrival)
{
  PositionController c({1, .2, 2, 1, .05, .05, .5});c.set_goal({{0,0}});
  flapping_uav_gcs::PositionControlResult out;
  for(int i=0;i<100;++i){out=c.step({{0,0}},{{.3,-.1}},.02,{{.3,-.1}},true);}
  EXPECT_TRUE(out.valid);EXPECT_FALSE(out.reached);EXPECT_NEAR(out.velocity[0],.3,1e-12);EXPECT_NEAR(out.velocity[1],-.1,1e-12);
  EXPECT_FALSE(c.step({{0,0}},{{0,0}},.02,{{NAN,0}},true).valid);
}
