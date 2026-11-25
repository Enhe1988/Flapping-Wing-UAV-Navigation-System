#include <gtest/gtest.h>
#include "flapping_uav_gcs/sample_order.hpp"
using namespace flapping_uav_gcs;  // NOLINT

TEST(SampleOrder, IsolatedOldPacketAndDuplicatesNeverReplaceLatestSample)
{
  SampleOrderFilter f;
  EXPECT_EQ(f.observe(1000000, 0, 0), SampleOrder::accepted);
  EXPECT_EQ(f.observe(1000000, 0, 10000000), SampleOrder::duplicate);
  EXPECT_EQ(f.observe(900000, 0, 20000000), SampleOrder::old_packet);
  EXPECT_EQ(f.last_time_us(), 1000000U);
  EXPECT_EQ(f.observe(1100000, 0, 100000000), SampleOrder::accepted);
  EXPECT_EQ(f.observe(950000, 0, 120000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(1200000, 0, 200000000), SampleOrder::accepted);
}

TEST(SampleOrder, SustainedAdvancingLowClockConfirmsReboot)
{
  SampleOrderFilter f;
  f.observe(500000000, 0, 0);
  EXPECT_EQ(f.observe(100000, 0, 100000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(120000, 0, 120000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(140000, 0, 140000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(160000, 0, 160000000), SampleOrder::clock_reset);
  EXPECT_EQ(f.last_time_us(), 160000U);
  EXPECT_EQ(f.observe(180000, 0, 180000000), SampleOrder::accepted);
}

TEST(SampleOrder, BurstOfBufferedOldSamplesDoesNotConfirmReboot)
{
  SampleOrderFilter f;
  f.observe(1000000, 0, 0);
  EXPECT_EQ(f.observe(600000, 0, 100000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(700000, 0, 101000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(800000, 0, 102000000), SampleOrder::old_packet);
  EXPECT_EQ(f.observe(1100000, 0, 110000000), SampleOrder::accepted);
}

TEST(SampleOrder, RepeatingOldTimestampIsNotAContinuingNewClock)
{
  SampleOrderFilter f;
  f.observe(1000000, 0, 0);
  for (int i = 1; i <= 10; ++i) {
    EXPECT_EQ(f.observe(500000, 0, i*100000000LL), SampleOrder::old_packet);
  }
  EXPECT_EQ(f.last_time_us(), 1000000U);
}

TEST(SampleOrder, ExplicitReferenceResetIsImmediate)
{
  SampleOrderFilter f;
  f.observe(1000000, 1, 0);
  EXPECT_EQ(f.observe(1000000, 2, 20000000), SampleOrder::clock_reset);
}

TEST(SampleOrder, HugeForwardOutlierDoesNotConvertNormalTrafficIntoAReset)
{
  SampleOrderFilter f;
  EXPECT_EQ(f.observe(117804000,0,0),SampleOrder::accepted);
  EXPECT_EQ(f.observe(6076276598041955504ULL,0,365000000),SampleOrder::future_packet);
  EXPECT_EQ(f.last_time_us(),117804000U);
  EXPECT_EQ(f.observe(118460000,0,930000000),SampleOrder::accepted);
  EXPECT_EQ(f.observe(118606000,0,1000000000),SampleOrder::accepted);
  EXPECT_EQ(f.last_time_us(),118606000U);
}

TEST(SampleOrder, HostTimeGapAndExplicitEpochChangeRemainSupported)
{
  SampleOrderFilter f;f.observe(1000000,0,0);
  EXPECT_EQ(f.observe(11000000,0,10000000000LL),SampleOrder::accepted);
  EXPECT_EQ(f.observe(13100000,0,10100000000LL),SampleOrder::accepted); // 2 s catch-up.
  EXPECT_EQ(f.observe(30000000,0,10200000000LL),SampleOrder::future_packet);
  EXPECT_EQ(f.observe(30000000,1,10300000000LL),SampleOrder::clock_reset);
}
