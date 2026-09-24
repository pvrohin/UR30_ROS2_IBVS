#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/calib3d.hpp>

#include "ur30_ibvs/panel_geometry.hpp"

namespace
{
using namespace ur30_ibvs;

constexpr double kPi = 3.14159265358979323846;
const cv::Matx33d kK(640, 0, 640, 0, 640, 360, 0, 0, 1);
constexpr double kTableZ = 0.805;  // panel top face in the world
constexpr double kLength = 0.9;
constexpr double kWidth = 0.55;

cv::Matx44d makePose(const cv::Matx33d & R, const cv::Vec3d & t)
{
  return {R(0, 0), R(0, 1), R(0, 2), t[0],
    R(1, 0), R(1, 1), R(1, 2), t[1],
    R(2, 0), R(2, 1), R(2, 2), t[2],
    0, 0, 0, 1};
}

cv::Matx33d rotZ(double a)
{
  return {std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1};
}

// The survey pose: 0.6 m above the panel, looking straight down, image x along world +X.
cv::Matx44d surveyPose()
{
  return makePose(cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1), {0.8, 0.0, 1.405});
}

// Image corners of a panel with the given centre and yaw, in PanelDetection order.
std::array<cv::Point2f, 4> cornersOf(
  const cv::Matx44d & world_T_cam, const cv::Vec3d & center, double yaw)
{
  const double hl = kLength / 2.0;
  const double hw = kWidth / 2.0;
  const cv::Vec3d local[4] = {{-hl, -hw, 0}, {hl, -hw, 0}, {hl, hw, 0}, {-hl, hw, 0}};
  std::array<cv::Point2f, 4> out;
  for (int i = 0; i < 4; ++i) {
    const cv::Vec3d p = center + rotZ(yaw) * local[i];
    const auto px = projectToPixel(kK, world_T_cam, p);
    out[i] = cv::Point2f(static_cast<float>(px->x), static_cast<float>(px->y));
  }
  return out;
}

void expectNear(const cv::Vec3d & a, const cv::Vec3d & b, double tol)
{
  EXPECT_LT(cv::norm(a - b), tol) << a << " vs " << b;
}
}  // namespace

TEST(PanelGeometry, ProjectionAndBackProjectionAreInverse)
{
  const cv::Vec3d p(0.95, 0.10, kTableZ);
  const auto px = projectToPixel(kK, surveyPose(), p);
  ASSERT_TRUE(px.has_value());
  const auto back = backProjectToPlane(
    kK, surveyPose(), cv::Point2f(static_cast<float>(px->x), static_cast<float>(px->y)), kTableZ);
  ASSERT_TRUE(back.has_value());
  expectNear(*back, p, 1e-4);  // pixel coordinates are float
}

TEST(PanelGeometry, ImpossibleGeometryGivesNothing)
{
  // A camera below the plane looking down never meets it.
  const cv::Matx44d below = makePose(cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1), {0.8, 0.0, 0.5});
  EXPECT_FALSE(backProjectToPlane(kK, below, cv::Point2f(640, 360), kTableZ).has_value());
  // A point behind the camera has no pixel.
  EXPECT_FALSE(projectToPixel(kK, surveyPose(), cv::Vec3d(0.8, 0.0, 2.0)).has_value());
}

TEST(PanelGeometry, RecoversPanelFromItsImageCorners)
{
  const cv::Vec3d centre(0.8, 0.0, kTableZ);
  const auto pose = panelPoseFromCorners(kK, surveyPose(), cornersOf(surveyPose(), centre, 0.0), kTableZ);
  ASSERT_TRUE(pose.has_value());
  expectNear(pose->center, centre, 1e-3);
  EXPECT_NEAR(pose->yaw, 0.0, 1e-3);
  EXPECT_NEAR(pose->length, kLength, 2e-3);
  EXPECT_NEAR(pose->width, kWidth, 2e-3);
}

TEST(PanelGeometry, RecoversAShiftedAndTurnedPanel)
{
  const cv::Vec3d centre(0.83, -0.02, kTableZ);
  const double yaw = 7.0 * kPi / 180.0;
  const auto pose = panelPoseFromCorners(kK, surveyPose(), cornersOf(surveyPose(), centre, yaw), kTableZ);
  ASSERT_TRUE(pose.has_value());
  expectNear(pose->center, centre, 1e-3);
  EXPECT_NEAR(pose->yaw, yaw, 1e-3);
}

TEST(PanelGeometry, HalfPixelCornerNoiseCostsUnderTwoMillimetres)
{
  // What the tilt-ambiguous PnP pose could not do: with 0.5 px of corner error
  // the back-projected panel stays within a couple of millimetres.
  const cv::Vec3d centre(0.8, 0.0, kTableZ);
  auto corners = cornersOf(surveyPose(), centre, 0.0);
  corners[0] += cv::Point2f(0.5f, -0.5f);
  corners[1] += cv::Point2f(-0.5f, -0.5f);
  corners[2] += cv::Point2f(0.5f, 0.5f);
  corners[3] += cv::Point2f(-0.5f, 0.5f);
  const auto pose = panelPoseFromCorners(kK, surveyPose(), corners, kTableZ);
  ASSERT_TRUE(pose.has_value());
  expectNear(pose->center, centre, 2e-3);
  EXPECT_LT(std::abs(pose->yaw), 0.2 * kPi / 180.0);
}

TEST(PanelGeometry, LongEdgesSitHalfAWidthEitherSideOfTheCentre)
{
  const PanelWorldPose panel{{0.8, 0.0, kTableZ}, 0.0, kLength, kWidth};
  const auto plus = longEdge(panel, kWidth, +1.0);
  expectNear(plus.midpoint, {0.8, 0.275, kTableZ}, 1e-12);
  expectNear(plus.direction, {1, 0, 0}, 1e-12);
  expectNear(longEdge(panel, kWidth, -1.0).midpoint, {0.8, -0.275, kTableZ}, 1e-12);

  // Turned by 90 degrees the "+y" edge moves to -x.
  const PanelWorldPose turned{{0.8, 0.0, kTableZ}, kPi / 2.0, kLength, kWidth};
  expectNear(longEdge(turned, kWidth, +1.0).midpoint, {0.8 - 0.275, 0.0, kTableZ}, 1e-12);
}

TEST(PanelGeometry, ApproachTargetIsAboveTheEdgeLookingDown)
{
  const PanelEdge edge{{0.8, 0.275, kTableZ}, {1, 0, 0}};
  const cv::Matx44d target = approachTarget(edge, 0.2, surveyPose());
  expectNear(translationOf(target), {0.8, 0.275, 1.005}, 1e-12);
  // Optical z straight down, image x along the edge.
  const cv::Matx33d R = rotationOf(target);
  expectNear({R(0, 2), R(1, 2), R(2, 2)}, {0, 0, -1}, 1e-12);
  expectNear({R(0, 0), R(1, 0), R(2, 0)}, {1, 0, 0}, 1e-12);
  // Right-handed.
  EXPECT_NEAR(cv::determinant(R), 1.0, 1e-12);
}

TEST(PanelGeometry, ApproachTargetNeverTurnsTheCameraHalfARevolution)
{
  // The edge direction is only defined up to sign; the target must keep the image
  // x axis on the side it already points to.
  const PanelEdge reversed{{0.8, 0.275, kTableZ}, {-1, 0, 0}};
  const cv::Matx33d R = rotationOf(approachTarget(reversed, 0.2, surveyPose()));
  expectNear({R(0, 0), R(1, 0), R(2, 0)}, {1, 0, 0}, 1e-12);
}

TEST(PanelGeometry, ApproachTwistPointsTowardsTheTargetInTheCameraFrame)
{
  const PanelEdge edge{{0.8, 0.275, kTableZ}, {1, 0, 0}};
  const cv::Matx44d target = approachTarget(edge, 0.2, surveyPose());
  const auto cmd = approachTwist(surveyPose(), target, 1.0, 1.0, 0.05, 0.1);

  // World error is (0, +0.275, -0.4). The camera looks down with y = world -Y and
  // z = world -Z, so it must move to -y and +z in its own frame.
  EXPECT_LT(cmd.twist[1], 0.0);
  EXPECT_GT(cmd.twist[2], 0.0);
  EXPECT_NEAR(cmd.twist[0], 0.0, 1e-12);
  EXPECT_NEAR(std::hypot(std::hypot(cmd.twist[0], cmd.twist[1]), cmd.twist[2]), 0.05, 1e-9);
  EXPECT_NEAR(cmd.position_error, std::hypot(0.275, 0.4), 1e-12);
  EXPECT_NEAR(cmd.rotation_error, 0.0, 1e-9);
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(cmd.twist[i], 0.0, 1e-9);
  }
}

TEST(PanelGeometry, ApproachTwistTurnsTowardsTheTargetOrientation)
{
  // Target turned +20 degrees about world z. The camera's z is world -Z, so the
  // same turn is about -z in the camera frame.
  const cv::Matx44d current = surveyPose();
  const cv::Matx44d target =
    makePose(rotZ(20.0 * kPi / 180.0) * rotationOf(current), translationOf(current));
  const auto cmd = approachTwist(current, target, 1.0, 1.0, 0.05, 1.0);
  EXPECT_NEAR(cmd.twist[5], -20.0 * kPi / 180.0, 1e-9);
  EXPECT_NEAR(cmd.rotation_error, 20.0 * kPi / 180.0, 1e-9);
  EXPECT_NEAR(cmd.position_error, 0.0, 1e-12);
}

TEST(PanelGeometry, TwistInParentAddsTheLeverArm)
{
  // Camera 0.1 m ahead of the tool frame, same axes. Turning the camera about its x
  // axis swings the tool origin sideways: v = t x w = (0, 0.1 w, 0).
  const cv::Matx44d tool_T_cam = makePose(cv::Matx33d::eye(), {0.0, 0.0, 0.1});
  const auto tw = twistInParent(tool_T_cam, {0, 0, 0, 0.5, 0, 0});
  EXPECT_NEAR(tw[0], 0.0, 1e-12);
  EXPECT_NEAR(tw[1], 0.05, 1e-12);
  EXPECT_NEAR(tw[2], 0.0, 1e-12);
  EXPECT_NEAR(tw[3], 0.5, 1e-12);
}

TEST(PanelGeometry, TwistInParentMatchesTheMotionOfARigidBody)
{
  // The camera and tool frame are rigidly linked, as on the wrist. Move the camera
  // with a camera-frame twist for a tiny dt, watch where the tool origin goes, and
  // compare with twistInParent.
  const cv::Matx44d tool_T_cam =
    makePose(rotZ(0.4) * cv::Matx33d(1, 0, 0, 0, 0, -1, 0, 1, 0), {0.01, -0.02, 0.03});
  const std::array<double, 6> twist{0.02, -0.03, 0.04, 0.1, -0.2, 0.3};

  const cv::Matx44d world_T_cam = makePose(rotZ(0.9), {0.5, -0.2, 1.0});
  const cv::Matx44d cam_T_tool = tool_T_cam.inv();
  const cv::Matx44d world_T_tool = world_T_cam * cam_T_tool;

  const double dt = 1e-6;
  cv::Matx33d dR;
  cv::Rodrigues(cv::Vec3d(twist[3], twist[4], twist[5]) * dt, dR);
  const cv::Matx44d moved = makePose(
    rotationOf(world_T_cam) * dR,
    translationOf(world_T_cam) + rotationOf(world_T_cam) * cv::Vec3d(twist[0], twist[1], twist[2]) * dt);
  const cv::Matx44d moved_tool = moved * cam_T_tool;

  const cv::Vec3d v_world = (translationOf(moved_tool) - translationOf(world_T_tool)) * (1.0 / dt);
  const cv::Vec3d v_in_tool = rotationOf(world_T_tool).t() * v_world;

  const auto tw = twistInParent(tool_T_cam, twist);
  expectNear({tw[0], tw[1], tw[2]}, v_in_tool, 1e-4);
}

TEST(PanelGeometry, IntegratingTheApproachTwistsArrivesAtTheTarget)
{
  // Move a camera by the commanded camera-frame twists (p += R v dt, R *= exp(w dt))
  // and check it converges, so the twist sign and frame convention are right.
  const PanelEdge edge{{0.8, 0.275, kTableZ}, {1, 0, 0}};
  const cv::Matx44d target = approachTarget(edge, 0.2, surveyPose());
  cv::Matx33d R = rotationOf(surveyPose()) * rotZ(15.0 * kPi / 180.0);  // start a little turned
  cv::Vec3d p = translationOf(surveyPose());

  const double dt = 0.02;
  ApproachCommand cmd{};
  for (int i = 0; i < 1500; ++i) {
    cmd = approachTwist(makePose(R, p), target, 1.0, 1.0, 0.05, 0.1);
    const cv::Vec3d v(cmd.twist[0], cmd.twist[1], cmd.twist[2]);
    const cv::Vec3d w(cmd.twist[3], cmd.twist[4], cmd.twist[5]);
    p += R * v * dt;
    cv::Matx33d dR;
    cv::Rodrigues(w * dt, dR);
    R = R * dR;
  }
  EXPECT_LT(cmd.position_error, 1e-3);
  EXPECT_LT(cmd.rotation_error, 1e-3);
}
