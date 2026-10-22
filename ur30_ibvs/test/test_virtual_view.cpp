#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <opencv2/core.hpp>

#include "ur30_ibvs/virtual_view.hpp"

namespace
{
using ur30_ibvs::quarterTurnsForDirection;
using ur30_ibvs::rotateImage;
using ur30_ibvs::rotateIntrinsics;
using ur30_ibvs::rotatePixel;
using ur30_ibvs::rotatedSize;
using ur30_ibvs::twistFromVirtual;

const cv::Matx33d kK(640, 0, 639.5, 0, 640, 359.5, 0, 0, 1);
const cv::Size kSize(1280, 720);

// Real-camera coordinates of the virtual camera's axes: column i is the virtual axis i.
cv::Matx33d axesOf(int quarter_turns)
{
  cv::Matx33d R;
  for (int i = 0; i < 3; ++i) {
    std::array<double, 6> e{};
    e[i] = 1.0;
    const auto v = twistFromVirtual(e, quarter_turns);
    R(0, i) = v[0];
    R(1, i) = v[1];
    R(2, i) = v[2];
  }
  return R;
}
}  // namespace

TEST(VirtualView, RotatedImageMovesAPixelWhereRotatePixelSays)
{
  for (int k = 0; k < 4; ++k) {
    cv::Mat image = cv::Mat::zeros(kSize, CV_8UC3);
    image.at<cv::Vec3b>(100, 300) = cv::Vec3b(255, 255, 255);
    const cv::Mat out = rotateImage(image, k);
    EXPECT_EQ(out.size(), rotatedSize(kSize, k));
    const cv::Point2d p = rotatePixel(cv::Point2d(300, 100), kSize, k);
    EXPECT_EQ(out.at<cv::Vec3b>(static_cast<int>(p.y), static_cast<int>(p.x)),
      cv::Vec3b(255, 255, 255)) << "k = " << k;
  }
}

TEST(VirtualView, QuarterTurnsAreModuloFour)
{
  EXPECT_EQ(rotatedSize(kSize, 5), rotatedSize(kSize, 1));
  EXPECT_EQ(rotatedSize(kSize, -1), rotatedSize(kSize, 3));
  const cv::Point2d a = rotatePixel(cv::Point2d(10, 20), kSize, 6);
  const cv::Point2d b = rotatePixel(cv::Point2d(10, 20), kSize, 2);
  EXPECT_NEAR(cv::norm(a - b), 0.0, 1e-9);
}

TEST(VirtualView, IntrinsicsAndTwistsAgreeWithTheRotatedPixels)
{
  // A scene point seen by the real camera, and by the virtual camera whose axes are
  // axesOf(k) in the real frame, must land on pixels related by rotatePixel.
  const cv::Vec3d P(0.07, -0.04, 0.5);
  const cv::Point2d real(kK(0, 0) * P[0] / P[2] + kK(0, 2), kK(1, 1) * P[1] / P[2] + kK(1, 2));
  for (int k = 0; k < 4; ++k) {
    const cv::Vec3d Pv = axesOf(k).t() * P;
    const cv::Matx33d Kv = rotateIntrinsics(kK, kSize, k);
    const cv::Point2d virt(Kv(0, 0) * Pv[0] / Pv[2] + Kv(0, 2), Kv(1, 1) * Pv[1] / Pv[2] + Kv(1, 2));
    const cv::Point2d expected = rotatePixel(real, kSize, k);
    EXPECT_NEAR(virt.x, expected.x, 1e-6) << "k = " << k;
    EXPECT_NEAR(virt.y, expected.y, 1e-6) << "k = " << k;
  }
}

TEST(VirtualView, RotationalPartOfATwistTurnsLikeTheLinearPart)
{
  const std::array<double, 6> twist{0.1, 0.2, 0.3, 0.01, 0.02, 0.03};
  const auto out = twistFromVirtual(twist, 1);
  EXPECT_NEAR(out[0], 0.2, 1e-12);
  EXPECT_NEAR(out[1], -0.1, 1e-12);
  EXPECT_NEAR(out[2], 0.3, 1e-12);
  EXPECT_NEAR(out[3], 0.02, 1e-12);
  EXPECT_NEAR(out[4], -0.01, 1e-12);
  EXPECT_NEAR(out[5], 0.03, 1e-12);
}

TEST(VirtualView, VirtualPlusXPointsWhereTheDirectionLookupSays)
{
  // Directions in the real image plane: right, up, left, down.
  const cv::Vec2d directions[4] = {{1, 0}, {0, -1}, {-1, 0}, {0, 1}};
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(quarterTurnsForDirection(directions[k][0], directions[k][1]), k);
    const auto v = twistFromVirtual({1, 0, 0, 0, 0, 0}, k);
    EXPECT_NEAR(v[0], directions[k][0], 1e-12) << "k = " << k;
    EXPECT_NEAR(v[1], directions[k][1], 1e-12) << "k = " << k;
  }
}

TEST(VirtualView, DirectionLookupPicksTheNearestQuarterTurn)
{
  EXPECT_EQ(quarterTurnsForDirection(1.0, 0.2), 0);
  EXPECT_EQ(quarterTurnsForDirection(-1.0, -0.3), 2);
  EXPECT_EQ(quarterTurnsForDirection(0.2, 1.0), 3);
}

TEST(VirtualView, CounterClockwiseTurnOfTheTravelAddsOneQuarterTurn)
{
  // The lap goes round counter-clockwise on screen: left, down, right, up -> k = 2, 3, 0, 1.
  EXPECT_EQ(quarterTurnsForDirection(-1, 0), 2);
  EXPECT_EQ(quarterTurnsForDirection(0, 1), 3);
  EXPECT_EQ(quarterTurnsForDirection(1, 0), 0);
  EXPECT_EQ(quarterTurnsForDirection(0, -1), 1);
}
