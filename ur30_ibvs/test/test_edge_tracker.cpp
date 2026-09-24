#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/core.hpp>

#include "ur30_ibvs/edge_tracker.hpp"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
using ur30_ibvs::EdgeTracker;
using ur30_ibvs::EdgeTrackerParams;

constexpr int kWidth = 1280;
constexpr int kHeight = 720;
constexpr double kPi = 3.14159265358979323846;
constexpr int kBright = 197;  // table
constexpr int kDark = 107;    // panel
const vpCameraParameters kCam(640, 640, 640, 360);

// A straight step edge through (640, y_at_centre) with the given tilt.
// dark_below: the panel (dark) is on the larger-y side of the edge.
cv::Mat render(double y_at_centre, double tilt_deg = 0.0, bool dark_below = true)
{
  cv::Mat image(kHeight, kWidth, CV_8UC3);
  const double slope = std::tan(tilt_deg * kPi / 180.0);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const bool below = y > y_at_centre + slope * (x - 640);
      const int v = (below == dark_below) ? kDark : kBright;
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(v, v, v);
    }
  }
  return image;
}

// Seed points near the two ends of the edge, offset from it by `offset` px.
std::pair<cv::Point2d, cv::Point2d> seeds(double y_at_centre, double tilt_deg, double offset)
{
  const double slope = std::tan(tilt_deg * kPi / 180.0);
  const auto at = [&](double x) {return cv::Point2d(x, y_at_centre + slope * (x - 640) + offset);};
  return {at(100), at(1180)};
}

// The step lies between pixel rows y and y+1, i.e. at y + 0.5. ViSP's moving
// edges land within ~1.5 px of it, towards the dark side.
constexpr double kEdgeTolerancePx = 2.5;
}  // namespace

TEST(EdgeTracker, FindsHorizontalEdgeAndReportsLineFeature)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(300, 0, 0);
  ASSERT_TRUE(tracker.init(render(300), a, b));

  EXPECT_NEAR(tracker.rhoTheta()[0], 300.5, kEdgeTolerancePx);
  EXPECT_NEAR(tracker.rhoTheta()[1], 0.0, 0.01);

  vpFeatureLine s;
  tracker.feature(kCam, s);
  // Normalised: the line is y = (300.5 - 360) / 640, so rho = y with theta = +pi/2.
  EXPECT_NEAR(s.getRho(), (300.5 - 360.0) / 640.0, kEdgeTolerancePx / 640.0);
  EXPECT_NEAR(s.getTheta(), kPi / 2.0, 0.01);
}

TEST(EdgeTracker, FeatureSignFollowsWhichSideIsDark)
{
  // The same horizontal edge with the polarity reversed flips theta to -pi/2 and
  // the sign of rho. Whoever sets the desired line must take this from the
  // measurement, not assume +pi/2.
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(300, 0, 0);
  ASSERT_TRUE(tracker.init(render(300, 0, false), a, b));

  vpFeatureLine s;
  tracker.feature(kCam, s);
  EXPECT_NEAR(s.getTheta(), -kPi / 2.0, 0.01);
  EXPECT_NEAR(s.getRho(), -(300.5 - 360.0) / 640.0, kEdgeTolerancePx / 640.0);
}

TEST(EdgeTracker, LineThroughImageCentreHasZeroRho)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(360, 0, 0);
  ASSERT_TRUE(tracker.init(render(360), a, b));
  vpFeatureLine s;
  tracker.feature(kCam, s);
  EXPECT_NEAR(s.getRho(), 0.0, kEdgeTolerancePx / 640.0);
}

TEST(EdgeTracker, RecoversTiltOfEdge)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(360, 5.0, 2.0);
  ASSERT_TRUE(tracker.init(render(360, 5.0), a, b));
  EXPECT_NEAR(tracker.rhoTheta()[1], -5.0 * kPi / 180.0, 0.01);
}

TEST(EdgeTracker, SnapsSeedsThatAreOffTheEdge)
{
  // ViSP's own initialisation fails for a seed only 4 px off the edge.
  for (const double offset : {-45.0, -20.0, -8.0, 4.0, 14.0, 45.0}) {
    EdgeTracker tracker{EdgeTrackerParams{}};
    const auto [a, b] = seeds(300, 0, offset);
    ASSERT_TRUE(tracker.init(render(300), a, b)) << "offset " << offset;
    EXPECT_NEAR(tracker.rhoTheta()[0], 300.5, kEdgeTolerancePx) << "offset " << offset;
  }
}

TEST(EdgeTracker, RejectsSeedBeyondSnapRadius)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(300, 0, 70);
  EXPECT_FALSE(tracker.init(render(300), a, b));
  EXPECT_FALSE(tracker.isTracking());
}

TEST(EdgeTracker, FollowsAMovingEdge)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(300, 0, 0);
  ASSERT_TRUE(tracker.init(render(300), a, b));
  for (const double row : {306.0, 312.0, 318.0, 324.0}) {
    ASSERT_TRUE(tracker.track(render(row))) << "row " << row;
    EXPECT_NEAR(tracker.rhoTheta()[0], row + 0.5, kEdgeTolerancePx) << "row " << row;
  }
}

TEST(EdgeTracker, LosesTrackingWhenTheEdgeDisappears)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const auto [a, b] = seeds(300, 0, 0);
  ASSERT_TRUE(tracker.init(render(300), a, b));
  const cv::Mat blank(kHeight, kWidth, CV_8UC3, cv::Scalar(150, 150, 150));
  EXPECT_FALSE(tracker.track(blank));
  EXPECT_FALSE(tracker.isTracking());
  EXPECT_FALSE(tracker.track(render(300)));  // stays lost until init() is called again
}

TEST(EdgeTracker, InitFailsWithoutAnyEdge)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const cv::Mat blank(kHeight, kWidth, CV_8UC3, cv::Scalar(150, 150, 150));
  EXPECT_FALSE(tracker.init(blank, {100, 300}, {1180, 300}));
}

TEST(EdgeTracker, RejectsUnsupportedImages)
{
  EdgeTracker tracker{EdgeTrackerParams{}};
  const cv::Mat sixteen_bit(kHeight, kWidth, CV_16UC1, cv::Scalar(100));
  EXPECT_FALSE(tracker.init(sixteen_bit, {100, 300}, {1180, 300}));
  EXPECT_FALSE(tracker.init(cv::Mat(), {100, 300}, {1180, 300}));
}
