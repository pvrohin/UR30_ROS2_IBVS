#include <gtest/gtest.h>

#include "ur30_ibvs/timing_stats.hpp"

namespace
{
using ur30_ibvs::TimingStats;

TEST(TimingStats, EmptyHasNoSummary)
{
  TimingStats stats;
  EXPECT_FALSE(stats.takeSummary().has_value());
}

TEST(TimingStats, SingleSample)
{
  TimingStats stats;
  stats.add(3.0);
  const auto s = stats.takeSummary();
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->count, 1u);
  EXPECT_DOUBLE_EQ(s->mean, 3.0);
  EXPECT_DOUBLE_EQ(s->p99, 3.0);
  EXPECT_DOUBLE_EQ(s->max, 3.0);
}

TEST(TimingStats, PercentileAndMaxOfAHundredSamples)
{
  TimingStats stats;
  for (int i = 100; i >= 1; --i) {
    stats.add(i);
  }
  const auto s = stats.takeSummary();
  ASSERT_TRUE(s.has_value());
  EXPECT_DOUBLE_EQ(s->mean, 50.5);
  EXPECT_DOUBLE_EQ(s->p99, 99.0);
  EXPECT_DOUBLE_EQ(s->max, 100.0);
}

TEST(TimingStats, TakingASummaryStartsAFreshWindow)
{
  TimingStats stats;
  stats.add(100.0);
  ASSERT_TRUE(stats.takeSummary().has_value());
  EXPECT_FALSE(stats.takeSummary().has_value());
  stats.add(1.0);
  EXPECT_DOUBLE_EQ(stats.takeSummary()->max, 1.0);
}
}  // namespace
