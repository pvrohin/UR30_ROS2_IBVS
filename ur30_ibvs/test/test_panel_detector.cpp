#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "ur30_ibvs/panel_detector.hpp"

namespace
{
using ur30_ibvs::PanelDetector;
using ur30_ibvs::PanelDetectorParams;

constexpr int kWidth = 1280;
constexpr int kHeight = 720;
constexpr double kPi = 3.14159265358979323846;
const cv::Matx33d kK(640, 0, 640, 0, 640, 360, 0, 0, 1);

// Panel pose at the survey position: camera 0.6 m above the top face, looking
// straight down, image x along panel x (so the panel z axis points at the camera).
cv::Matx33d surveyRotation() {return cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1);}

cv::Matx33d rotZ(double a)
{
  return cv::Matx33d(std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1);
}

cv::Matx33d rotX(double a)
{
  return cv::Matx33d(1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a));
}

cv::Vec3d toRvec(const cv::Matx33d & R)
{
  cv::Vec3d rvec;
  cv::Rodrigues(R, rvec);
  return rvec;
}

// Bright table with a dark filled panel outline drawn from the given pose.
cv::Mat render(const cv::Matx33d & R, const cv::Vec3d & t, double length = 0.9, double width = 0.55)
{
  cv::Mat image(kHeight, kWidth, CV_8UC3, cv::Scalar(170, 170, 170));
  const float hl = static_cast<float>(length / 2.0);
  const float hw = static_cast<float>(width / 2.0);
  const std::vector<cv::Point3f> corners{{-hl, -hw, 0.f}, {hl, -hw, 0.f}, {hl, hw, 0.f},
    {-hl, hw, 0.f}};
  std::vector<cv::Point2f> projected;
  cv::projectPoints(corners, toRvec(R), t, kK, cv::noArray(), projected);
  std::vector<cv::Point> polygon;
  for (const auto & p : projected) {
    polygon.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  cv::fillConvexPoly(image, polygon, cv::Scalar(38, 38, 43));
  return image;
}

double rotationErrorDeg(const cv::Vec3d & estimated_rvec, const cv::Matx33d & truth)
{
  cv::Matx33d R;
  cv::Rodrigues(estimated_rvec, R);
  return cv::norm(toRvec(R.t() * truth)) * 180.0 / kPi;
}

PanelDetector defaultDetector() {return PanelDetector(PanelDetectorParams{});}

}  // namespace

// Tilt tolerance: seen head-on, foreshortening is second order, so the ~0.3 px
// corner quantisation of these synthetic images already reads as ~2.7 deg of
// tilt (0.02 deg with noise-free corners). Depth and in-plane position stay
// well inside 1 cm.
constexpr double kTiltToleranceDeg = 4.0;

// A panel turned by 10 degrees needs 0.9*sin(10)+0.55*cos(10) = 0.70 m of
// image height. At the 0.6 m survey height the view is only 0.68 m tall, so the
// panel would hang off the image; 0.7 m keeps it fully visible (638 of 720 px).
const cv::Vec3d kRotatedPanelPosition(0.02, -0.01, 0.7);

std::vector<cv::Point2f> projectedCorners(const cv::Matx33d & R, const cv::Vec3d & t)
{
  const float hl = 0.45f;
  const float hw = 0.275f;
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    std::vector<cv::Point3f>{{-hl, -hw, 0.f}, {hl, -hw, 0.f}, {hl, hw, 0.f}, {-hl, hw, 0.f}},
    toRvec(R), t, kK, cv::noArray(), projected);
  return projected;
}

TEST(PanelDetector, RecoversFrontalSurveyPose)
{
  const cv::Vec3d t(0, 0, 0.6);
  const auto detection = defaultDetector().detect(render(surveyRotation(), t), kK);
  ASSERT_TRUE(detection.has_value());
  EXPECT_LT(cv::norm(detection->tvec - t), 0.01);
  EXPECT_LT(rotationErrorDeg(detection->rvec, surveyRotation()), kTiltToleranceDeg);
  EXPECT_LT(detection->reprojection_error_px, 3.0);
}

TEST(PanelDetector, ReturnsCornersInPanelCornerOrder)
{
  const cv::Vec3d t = kRotatedPanelPosition;
  const cv::Matx33d R = surveyRotation() * rotZ(10.0 * kPi / 180.0);
  PanelDetectorParams params;
  params.expected_x_axis_angle_deg = -10.0;  // +10 deg about panel z points x at -10 deg in the image
  const auto detection = PanelDetector(params).detect(render(R, t), kK);
  ASSERT_TRUE(detection.has_value());
  const auto expected = projectedCorners(R, t);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_LT(cv::norm(detection->corners_px[i] - expected[i]), 1.5) << "corner " << i;
  }
}

TEST(PanelDetector, RecoversShiftedTiltedAndTurnedPose)
{
  const cv::Matx33d R = rotX(0.12) * surveyRotation() * rotZ(10.0 * kPi / 180.0);
  const cv::Vec3d t = kRotatedPanelPosition;
  PanelDetectorParams params;
  params.expected_x_axis_angle_deg = -10.0;  // +10 deg about panel z points x at -10 deg in the image
  const auto detection = PanelDetector(params).detect(render(R, t), kK);
  ASSERT_TRUE(detection.has_value());
  EXPECT_LT(cv::norm(detection->tvec - t), 0.015);
  EXPECT_LT(rotationErrorDeg(detection->rvec, R), kTiltToleranceDeg);
}

TEST(PanelDetector, ExpectedDirectionResolvesHalfTurnAmbiguity)
{
  // The image is identical for the pose and for the pose turned 180 degrees in
  // plane, so only the expected x-axis direction can choose between them.
  const cv::Vec3d t(0, 0, 0.6);
  const cv::Mat image = render(surveyRotation(), t);

  PanelDetectorParams right;
  right.expected_x_axis_angle_deg = 0.0;
  const auto a = PanelDetector(right).detect(image, kK);
  ASSERT_TRUE(a.has_value());
  EXPECT_LT(rotationErrorDeg(a->rvec, surveyRotation()), kTiltToleranceDeg);

  PanelDetectorParams left;
  left.expected_x_axis_angle_deg = 180.0;
  const auto b = PanelDetector(left).detect(image, kK);
  ASSERT_TRUE(b.has_value());
  EXPECT_LT(rotationErrorDeg(b->rvec, surveyRotation() * rotZ(kPi)), kTiltToleranceDeg);
}

TEST(PanelDetector, RejectsImageWithoutPanel)
{
  const cv::Mat bright(kHeight, kWidth, CV_8UC3, cv::Scalar(170, 170, 170));
  EXPECT_FALSE(defaultDetector().detect(bright, kK).has_value());
}

TEST(PanelDetector, RejectsWrongAspectRatio)
{
  // Square, fully inside the image and above the minimum area fraction (0.37 vs
  // 0.35), so only the aspect check can reject it.
  const cv::Mat square = render(surveyRotation(), cv::Vec3d(0, 0, 0.6), 0.55, 0.55);
  EXPECT_FALSE(defaultDetector().detect(square, kK).has_value());
}

TEST(PanelDetector, RejectsNonColourImage)
{
  const cv::Mat gray(kHeight, kWidth, CV_8UC1, cv::Scalar(30));
  EXPECT_FALSE(defaultDetector().detect(gray, kK).has_value());
}
