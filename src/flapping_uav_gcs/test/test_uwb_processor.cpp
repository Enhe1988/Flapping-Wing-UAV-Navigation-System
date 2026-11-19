#include <cmath>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/uwb_processor.hpp"

using namespace flapping_uav_gcs;  // NOLINT

UwbConfig confirmed()
{
  UwbConfig config; config.coordinates_confirmed = true; return config;
}
UwbFrame frame(uint32_t time, double x = 1, double y = 2, double z = .8)
{
  UwbFrame result; result.anchor_id = 7; result.local_time = result.system_time = time;
  result.tags = {UwbTag{0, 99, {{99, 99, 99}}}, UwbTag{0, 0, {{x, y, z}}}};
  return result;
}

TEST(UwbProcessor, RejectsInvalidConfiguration)
{
  auto config = confirmed(); config.tag_id = 256;
  EXPECT_THROW(UwbProcessor{config}, std::invalid_argument);
  config = confirmed(); config.timeout_s = 0;
  EXPECT_THROW(UwbProcessor{config}, std::invalid_argument);
  config = confirmed(); config.translation[0] = unknown();
  EXPECT_THROW(UwbProcessor{config}, std::invalid_argument);
  config = confirmed(); config.measured_stddev_xy_m = 0;
  EXPECT_THROW(UwbProcessor{config}, std::invalid_argument);
  config = confirmed(); config.y_sign = 0;
  EXPECT_THROW(UwbProcessor{config}, std::invalid_argument);
}
TEST(UwbProcessor, StartsInvalidAndNeverFabricatesAStamp)
{
  UwbProcessor model(confirmed()); const auto state = model.snapshot(100);
  EXPECT_EQ(state.state, UwbState::no_data); EXPECT_FALSE(state.position_valid);
  EXPECT_TRUE(std::isnan(state.frame_age_s)); EXPECT_EQ(state.accepted_positions, 0U);
}
TEST(UwbProcessor, RequiresExplicitCoordinateConfirmation)
{
  UwbProcessor model(UwbConfig{});
  EXPECT_FALSE(model.process(frame(1), 0));
  EXPECT_EQ(model.snapshot(0).state, UwbState::coordinates_unconfirmed);
  EXPECT_DOUBLE_EQ(model.snapshot(0).raw_position[0], 1);
}
TEST(UwbProcessor, SelectsTagZeroByIdNotArrayIndex)
{
  UwbProcessor model(confirmed()); ASSERT_TRUE(model.process(frame(1), 0));
  const auto state = model.snapshot(.1);
  EXPECT_TRUE(state.position_valid); EXPECT_DOUBLE_EQ(state.map_position[0], 1);
  EXPECT_DOUBLE_EQ(state.map_position[1], 2); EXPECT_DOUBLE_EQ(state.map_position[2], .8);
  EXPECT_FALSE(state.covariance_valid); EXPECT_TRUE(std::isnan(state.covariance_xy[0]));
}
TEST(UwbProcessor, AppliesScaleYawAndTranslation)
{
  auto config = confirmed(); config.position_scale = 2;
  config.yaw_rad = std::acos(-1) / 2; config.translation = {{3, 4, 5}};
  UwbProcessor model(config); ASSERT_TRUE(model.process(frame(1), 0));
  const auto p = model.snapshot(0).map_position;
  EXPECT_NEAR(p[0], -1, 1e-9); EXPECT_NEAR(p[1], 6, 1e-9); EXPECT_NEAR(p[2], 6.6, 1e-9);
}
TEST(UwbProcessor, ReflectsNativeRightSideYBeforeRotationAndPreservesRawData)
{
  auto config = confirmed(); config.y_sign = -1; config.position_scale = 2;
  config.yaw_rad = std::acos(-1) / 2; config.translation = {{3, 4, 5}};
  UwbProcessor model(config); ASSERT_TRUE(model.process(frame(1), 0));
  auto state = model.snapshot(0);
  EXPECT_NEAR(state.map_position[0], 7, 1e-9);
  EXPECT_NEAR(state.map_position[1], 6, 1e-9);
  EXPECT_NEAR(state.map_position[2], 6.6, 1e-9);
  EXPECT_DOUBLE_EQ(state.raw_position[1], 2);
  config = confirmed(); config.y_sign = -1; config.collect_static_noise = true;
  config.noise_sample_limit = 3;
  UwbProcessor noise(config);
  ASSERT_TRUE(noise.process(frame(1,0,0),0));
  ASSERT_TRUE(noise.process(frame(2,1,2),.05));
  ASSERT_TRUE(noise.process(frame(3,2,4),.1));
  state = noise.snapshot(.1); ASSERT_TRUE(state.covariance_valid);
  EXPECT_NEAR(state.covariance_xy[0],1,1e-12);
  EXPECT_NEAR(state.covariance_xy[1],-2,1e-12);
  EXPECT_NEAR(state.covariance_xy[2],-2,1e-12);
  EXPECT_NEAR(state.covariance_xy[3],4,1e-12);
}
TEST(UwbProcessor, OptionalRoleFilterAndAmbiguity)
{
  auto config = confirmed(); config.tag_role = 4; UwbProcessor model(config);
  EXPECT_FALSE(model.process(frame(1), 0));
  auto f = frame(2); f.tags[1].role = 4; EXPECT_TRUE(model.process(f, .05));
  f = frame(3); f.tags[1].role = 4; f.tags.push_back(f.tags[1]);
  EXPECT_FALSE(model.process(f, .1));
  EXPECT_EQ(model.snapshot(.1).state, UwbState::ambiguous_target);
}
TEST(UwbProcessor, MissingTagInvalidatesAndRecovers)
{
  UwbProcessor model(confirmed()); model.process(frame(1), 0);
  auto f = frame(2); f.tags.erase(f.tags.begin() + 1);
  EXPECT_FALSE(model.process(f, .05));
  EXPECT_EQ(model.snapshot(.05).state, UwbState::target_missing);
  EXPECT_EQ(model.snapshot(.05).accepted_positions, 1U);
  EXPECT_TRUE(model.process(frame(3), .1)); EXPECT_TRUE(model.snapshot(.1).position_valid);
}
TEST(UwbProcessor, RejectsNonfiniteXYButDoesNotInventUnknownZ)
{
  UwbProcessor model(confirmed());
  EXPECT_FALSE(model.process(frame(1, unknown()), 0));
  EXPECT_FALSE(model.process(frame(2, 1, std::numeric_limits<double>::infinity()), .05));
  EXPECT_TRUE(model.process(frame(3, 1, 2, unknown()), .1));
  EXPECT_TRUE(std::isnan(model.snapshot(.1).map_position[2]));
  EXPECT_TRUE(model.process(frame(4, 1, 2, std::numeric_limits<double>::infinity()), .15));
  EXPECT_TRUE(std::isnan(model.snapshot(.15).map_position[2]));
}
TEST(UwbProcessor, RepeatedFramesDoNotKeepOldPositionFresh)
{
  UwbProcessor model(confirmed()); ASSERT_TRUE(model.process(frame(1), 0));
  for (int i = 1; i <= 6; ++i) {
    EXPECT_FALSE(model.process(frame(1), i * .1));
  }
  const auto state = model.snapshot(.6);
  EXPECT_EQ(state.state, UwbState::duplicate_stale); EXPECT_FALSE(state.position_valid);
  EXPECT_EQ(state.accepted_positions, 1U); EXPECT_EQ(state.duplicate_frames, 6U);
  EXPECT_NEAR(state.position_age_s, .6, 1e-9); EXPECT_NEAR(state.frame_age_s, 0, 1e-9);
}
TEST(UwbProcessor, TimeoutAndIdenticalStartupStampAfterDisconnectRecover)
{
  UwbProcessor model(confirmed()); model.process(frame(0), 0);
  EXPECT_EQ(model.snapshot(.501).state, UwbState::stream_timeout);
  EXPECT_TRUE(model.process(frame(0), .6)); EXPECT_TRUE(model.snapshot(.6).position_valid);
}
TEST(UwbProcessor, WrapAndDeviceTimeBackwardsContinue)
{
  UwbProcessor model(confirmed());
  EXPECT_TRUE(model.process(frame(0xfffffff0), 0));
  EXPECT_TRUE(model.process(frame(10), .05));
  EXPECT_EQ(model.snapshot(.05).device_time_backwards, 0U);
  EXPECT_TRUE(model.process(frame(5000), .1));
  EXPECT_TRUE(model.process(frame(0), .15));
  EXPECT_EQ(model.snapshot(.15).device_time_backwards, 1U);
}
TEST(UwbProcessor, RejectsJumpsAndExplicitResetClearsTheReference)
{
  auto config = confirmed(); config.max_jump_m = .5;
  UwbProcessor model(config); model.process(frame(1), 0);
  EXPECT_FALSE(model.process(frame(2, 10), .05));
  EXPECT_EQ(model.snapshot(.05).state, UwbState::position_jump);
  model.reset_observation(); EXPECT_TRUE(model.process(frame(3, 10), .1));
  EXPECT_EQ(model.snapshot(.1).jump_rejections, 1U);
}
TEST(UwbProcessor, MeasuresStaticCovarianceAndFreezesItBeforeMovement)
{
  auto config = confirmed(); config.collect_static_noise = true; config.noise_sample_limit = 3;
  UwbProcessor model(config);
  model.process(frame(1, 0, 0), 0); model.process(frame(2, 1, 2), .05);
  EXPECT_FALSE(model.snapshot(.05).covariance_valid);
  model.process(frame(3, 2, 4), .1); const auto result = model.snapshot(.1);
  ASSERT_TRUE(result.covariance_valid);
  EXPECT_DOUBLE_EQ(result.covariance_xy[0], 1); EXPECT_DOUBLE_EQ(result.covariance_xy[1], 2);
  EXPECT_DOUBLE_EQ(result.covariance_xy[3], 4);
  model.process(frame(4, 100, 100), .15);
  EXPECT_EQ(model.snapshot(.15).static_noise_samples, 3U);
  EXPECT_DOUBLE_EQ(model.snapshot(.15).covariance_xy[0], 1);
}
TEST(UwbProcessor, ZeroVarianceDoesNotClaimPerfectMeasurement)
{
  auto config = confirmed(); config.collect_static_noise = true; config.noise_sample_limit = 2;
  UwbProcessor model(config); model.process(frame(1), 0); model.process(frame(2), .05);
  EXPECT_FALSE(model.snapshot(.05).covariance_valid);
  EXPECT_TRUE(std::isnan(model.snapshot(.05).covariance_xy[0]));
}
TEST(UwbProcessor, SupportsMeasuredNoiseAndCountsAcceptedFrequency)
{
  auto config = confirmed(); config.measured_stddev_xy_m = .2;
  UwbProcessor model(config);
  for (int i = 0; i < 5; ++i) {
    model.process(frame(i), i * .05);
  }
  EXPECT_NEAR(model.snapshot(.2).accepted_rate_hz, 20, 1e-9);
  EXPECT_NEAR(model.snapshot(.2).covariance_xy[0], .04, 1e-9);
  EXPECT_EQ(model.snapshot(1).accepted_rate_hz, 0);
}
