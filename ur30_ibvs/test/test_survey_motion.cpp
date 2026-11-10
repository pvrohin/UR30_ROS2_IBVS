#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "ur30_ibvs/survey_motion.hpp"

namespace
{
using ur30_ibvs::largestJointMove;
using ur30_ibvs::surveyDuration;
using ur30_ibvs::SurveyLimits;

const SurveyLimits kLimits{6.0, 0.5, 3.0};

TEST(SurveyMotion, LargestMoveIsTheBiggestAbsoluteChange)
{
  EXPECT_DOUBLE_EQ(*largestJointMove({0.0, 1.0, -1.0}, {0.5, -1.0, -1.25}), 2.0);
}

TEST(SurveyMotion, ShortMoveKeepsTheMinimumDuration)
{
  EXPECT_DOUBLE_EQ(*surveyDuration({0.0, 0.0}, {1.0, 0.0}, kLimits), 6.0);
}

TEST(SurveyMotion, LongMoveIsSlowedToTheSpeedLimit)
{
  // 2.5 rad at 0.5 rad/s, with a shorter minimum so that the speed limit governs.
  EXPECT_DOUBLE_EQ(*surveyDuration({0.0}, {2.5}, SurveyLimits{1.0, 0.5, 3.0}), 5.0);
}

TEST(SurveyMotion, MoveBeyondTheLimitIsRefused)
{
  EXPECT_FALSE(surveyDuration({0.0}, {3.1}, kLimits).has_value());
  EXPECT_TRUE(surveyDuration({0.0}, {3.0}, kLimits).has_value());
}

TEST(SurveyMotion, DirectionDoesNotMatter)
{
  EXPECT_FALSE(surveyDuration({3.1}, {0.0}, kLimits).has_value());
}

TEST(SurveyMotion, InvalidInputsAreRefused)
{
  EXPECT_FALSE(surveyDuration({}, {}, kLimits).has_value());
  EXPECT_FALSE(surveyDuration({0.0}, {0.0, 0.0}, kLimits).has_value());
  EXPECT_FALSE(
    surveyDuration({std::numeric_limits<double>::quiet_NaN()}, {0.0}, kLimits).has_value());
  EXPECT_FALSE(surveyDuration({0.0}, {0.1}, SurveyLimits{6.0, 0.0, 3.0}).has_value());
}
}  // namespace
