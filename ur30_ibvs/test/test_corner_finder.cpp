#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ur30_ibvs/corner_finder.hpp"

namespace
{
using ur30_ibvs::CornerFinder;
using ur30_ibvs::CornerFinderParams;

constexpr int kWidth = 1280;
constexpr int kHeight = 720;
constexpr int kBright = 197;  // table
constexpr int kDark = 107;    // panel

// The part is the dark region on the `below` (or above) side of a line through
// (640, y_at_centre) with slope `slope` (rows per column), for columns < corner_col.
cv::Mat render(double y_at_centre, double slope, double corner_col, bool below = true)
{
  cv::Mat image(kHeight, kWidth, CV_8UC3, cv::Scalar(kBright, kBright, kBright));
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const bool side = y > y_at_centre + slope * (x - 640);
      if (side == below && x < corner_col) {
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(kDark, kDark, kDark);
      }
    }
  }
  return image;
}

// ViSP pixel convention: i cos(theta) + j sin(theta) = rho (i = row, j = column).
void lineOf(double y_at_centre, double slope, double & rho, double & theta)
{
  theta = -std::atan(slope);
  rho = (y_at_centre - slope * 640.0) * std::cos(theta);
}

std::optional<double> find(const cv::Mat & image, double y, double slope, double from_col)
{
  double rho, theta;
  lineOf(y, slope, rho, theta);
  return CornerFinder(CornerFinderParams{}).find(image, rho, theta, from_col);
}
}  // namespace

TEST(CornerFinder, FindsTheColumnWhereThePartEnds)
{
  const auto corner = find(render(360, 0.0, 800), 360, 0.0, 400);
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(*corner, 800.0, 1.5);
}

TEST(CornerFinder, WorksWhenThePartIsAboveTheEdge)
{
  const auto corner = find(render(360, 0.0, 700, false), 360, 0.0, 400);
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(*corner, 700.0, 1.5);
}

TEST(CornerFinder, FollowsATiltedEdge)
{
  const double slope = std::tan(2.0 * M_PI / 180.0);
  const auto corner = find(render(340, slope, 900), 340, slope, 400);
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(*corner, 900.0, 2.0);
}

TEST(CornerFinder, ReportsNothingWhileTheCornerIsOutOfView)
{
  EXPECT_FALSE(find(render(360, 0.0, 5000), 360, 0.0, 400).has_value());
}

TEST(CornerFinder, ReportsNothingWithoutAPartBesideTheEdge)
{
  // Right of the corner everything is table: nothing dark to read.
  EXPECT_FALSE(find(render(360, 0.0, 300), 360, 0.0, 400).has_value());
}

TEST(CornerFinder, ReportsNothingForAnUnusableLineOrImage)
{
  const cv::Mat image = render(360, 0.0, 800);
  EXPECT_FALSE(CornerFinder(CornerFinderParams{}).find(image, 640.0, M_PI / 2.0, 400).has_value());
  EXPECT_FALSE(CornerFinder(CornerFinderParams{}).find(cv::Mat(), 360.0, 0.0, 400).has_value());
  const cv::Mat gray(kHeight, kWidth, CV_8UC1, cv::Scalar(100));
  EXPECT_FALSE(CornerFinder(CornerFinderParams{}).find(gray, 360.0, 0.0, 400).has_value());
}

TEST(CornerFinder, IgnoresATextureInsideThePart)
{
  // An armrest-like dark patch is still part: no early corner.
  cv::Mat image = render(360, 0.0, 800);
  cv::rectangle(image, cv::Rect(500, 420, 100, 60), cv::Scalar(60, 60, 60), cv::FILLED);
  const auto corner = find(image, 360, 0.0, 400);
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(*corner, 800.0, 1.5);
}

TEST(CornerFinder, ABrightStretchAtTheStartIsNotACorner)
{
  // The strip starts on the table (just beyond the previous corner, or the part has been
  // moved there) and the part begins 15 px later: the corner is still the far end.
  cv::Mat image = render(360, 0.0, 800);
  cv::rectangle(image, cv::Rect(300, 0, 35, kHeight), cv::Scalar(kBright, kBright, kBright),
    cv::FILLED);
  const auto corner = find(image, 360, 0.0, 320);
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(*corner, 800.0, 1.5);
}
