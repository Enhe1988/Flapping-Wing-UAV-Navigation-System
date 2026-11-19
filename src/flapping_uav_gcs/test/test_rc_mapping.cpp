#include <limits>
#include <vector>
#include <gtest/gtest.h>
#include "flapping_uav_gcs/rc_mapping.hpp"
#include "mavlink_codec.hpp"

using flapping_uav_gcs::RcMapping;

TEST(RcMapping, UserChannelOrderAndPhysicalStickEndpoints)
{
  RcMapping mapping;
  mapping.validate();
  std::array<uint16_t, 18> channels;
  // Move one physical axis at a time: a2 is left Y, a3 is left X.
  struct Endpoint {int axis; float raw; int channel; uint16_t expected;};
  const std::vector<Endpoint> cases{
    {0, 1, 0, 1000}, {0, -1, 0, 2000},  // right X: left/right
    {1, 1, 1, 1000}, {1, -1, 1, 2000},  // right Y: bottom/top (live correction)
    {2, 1, 2, 1000}, {2, -1, 2, 2000},  // left Y: bottom/top
    {3, 1, 3, 1000}, {3, -1, 3, 2000}}; // left X: left/right
  for (const auto & endpoint : cases) {
    std::vector<float> axes{0, 0, 0, 0, 1, 1, 1, 1};
    axes[endpoint.axis] = endpoint.raw;
    ASSERT_TRUE(mapping.convert(axes, channels));
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(channels[i], i == endpoint.channel ? endpoint.expected : 1500);
    }
  }
  ASSERT_TRUE(mapping.convert({0, 0, 0, 0, 1, 1, 1, 1}, channels));
  for (int i = 4; i < 8; ++i) {EXPECT_EQ(channels[i], 1000);}
  ASSERT_TRUE(mapping.convert({0, 0, 0, 0, -1, 0, 0, -1}, channels));
  EXPECT_EQ(channels[4], 2000); EXPECT_EQ(channels[7], 2000);
  EXPECT_EQ(channels[5], 1500); EXPECT_EQ(channels[6], 1500);
  ASSERT_TRUE(mapping.convert({0, 0, 0, 0, 1, -1, -1, 1}, channels));
  EXPECT_EQ(channels[5], 2000); EXPECT_EQ(channels[6], 2000);
  EXPECT_EQ(channels[8], 1000); EXPECT_EQ(channels[10], 1000);
  EXPECT_EQ(channels[16], 65535); EXPECT_EQ(channels[17], 65535);
}

TEST(RcMapping, DeadzoneAndValidation)
{
  RcMapping mapping;
  std::array<uint16_t, 18> channels;
  ASSERT_TRUE(mapping.convert({.01, -.01, 0, 0, 0, 0, 0, 0}, channels));
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(channels[i], 1500);
  }
  EXPECT_FALSE(mapping.convert({0, 0}, channels));
  EXPECT_FALSE(
    mapping.convert(
      {0, 0, std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0, 0},
      channels));
  EXPECT_FALSE(mapping.convert({2, 0, 0, 0, 0, 0, 0, 0}, channels));
  mapping.indices[0] = -1;
  EXPECT_THROW(mapping.validate(), std::invalid_argument);
  mapping = RcMapping(); mapping.signs[0] = 0;
  EXPECT_THROW(mapping.validate(), std::invalid_argument);
}

TEST(MavlinkCodec, ExtensionZeroPaddingAndNonfiniteValues)
{
  mavlink_message_t message;
  mavlink_msg_rc_channels_override_pack(
    255, 190, &message, 1, 1,
    1000, 1500, 1000, 1500, 1000, 1500, 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  auto text = flapping_uav_gcs::message_json(message);
  EXPECT_NE(text.find("\"chan18_raw\":0"), std::string::npos);
  mavlink_msg_named_value_float_pack(1, 1, &message, 42, "height", NAN);
  text = flapping_uav_gcs::message_json(message);
  EXPECT_NE(text.find("\"name\":\"height\""), std::string::npos);
  EXPECT_NE(text.find("\"value\":\"nan\""), std::string::npos);
}
