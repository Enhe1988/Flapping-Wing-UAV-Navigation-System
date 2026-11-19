#include <limits>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/position_controller.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"

using namespace flapping_uav_gcs;  // NOLINT

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
