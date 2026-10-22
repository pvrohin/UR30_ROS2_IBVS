#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>

#include "ur30_ibvs/ibvs_controller.hpp"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
using ur30_ibvs::IbvsController;
using ur30_ibvs::IbvsControllerParams;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.02;
constexpr int kSteps = 700;  // 14 s; the loop's time constant is ~1 / lambda = 1.7 s
constexpr double kStandoff = 0.2;

// The edge is the object x axis (y = 0, z = 0). The camera looks straight down at
// the object plane from height `standoff`, at lateral position y_c across the
// edge, with the survey orientation (image x along the edge), then turned by
// `yaw` about its optical axis.
vpHomogeneousMatrix cameraAbove(double y_c, double standoff, double yaw = 0.0)
{
  return vpHomogeneousMatrix(0, 0, 0, 0, 0, yaw) * vpHomogeneousMatrix(0, y_c, standoff, kPi, 0, 0);
}

// The edge as the controller would receive it from the image.
std::pair<double, double> measure(const vpHomogeneousMatrix & cMo)
{
  vpLine line;
  line.setWorldCoordinates(0, 1, 0, 0, 0, 0, 1, 0);
  line.changeFrame(cMo);
  line.projection();
  vpFeatureLine s;
  vpFeatureBuilder::create(s, line);
  return {s.getRho(), s.getTheta()};
}

struct Outcome
{
  double rho, theta;
  double lateral;          // camera position across the edge, in the object frame [m]
  double height;           // camera height above the object plane [m]
  double integrated_wz;    // total rotation commanded about the optical axis [rad]
  bool only_vy_wz_nonzero;
};

Outcome simulate(IbvsController & controller, vpHomogeneousMatrix cMo)
{
  Outcome out{};
  out.only_vy_wz_nonzero = true;
  for (int i = 0; i < kSteps; ++i) {
    const auto [rho, theta] = measure(cMo);
    const auto twist = controller.computeTwist(rho, theta, cMo[2][3]);
    out.only_vy_wz_nonzero = out.only_vy_wz_nonzero && twist[0] == 0.0 && twist[2] == 0.0 &&
      twist[3] == 0.0 && twist[4] == 0.0;
    out.integrated_wz += twist[5] * kDt;
    vpColVector v(6);
    for (unsigned int k = 0; k < 6; ++k) {
      v[k] = twist[k];
    }
    cMo = vpExponentialMap::direct(v, kDt).inverse() * cMo;
  }
  const auto [rho, theta] = measure(cMo);
  const vpHomogeneousMatrix oMc = cMo.inverse();
  out.rho = rho;
  out.theta = theta;
  out.lateral = oMc[1][3];
  out.height = oMc[2][3];
  return out;
}
}  // namespace

TEST(IbvsController, FirstCommandMovesTheCameraTowardsTheEdge)
{
  // Camera 3 cm to the -y side of the edge, so the edge appears above the image
  // centre (rho = (-0.03) / 0.2 = -0.15). The controller must move the camera to
  // +y, i.e. towards the edge, which is -y in the camera frame.
  const vpHomogeneousMatrix cMo = cameraAbove(-0.03, kStandoff);
  const auto [rho, theta] = measure(cMo);
  ASSERT_NEAR(rho, -0.15, 1e-9);
  ASSERT_NEAR(theta, kPi / 2.0, 1e-9);

  IbvsController controller{IbvsControllerParams{}};
  const auto twist = controller.computeTwist(rho, theta, kStandoff);
  // vy = lambda * distance * rho_error = 0.6 * 0.2 * (-0.15)
  EXPECT_NEAR(twist[1], -0.018, 1e-6);
  EXPECT_NEAR(twist[5], 0.0, 1e-9);
}

TEST(IbvsController, CentresTheEdgeFromALateralOffset)
{
  IbvsController controller{IbvsControllerParams{}};
  const auto out = simulate(controller, cameraAbove(-0.03, kStandoff));
  EXPECT_NEAR(out.rho, 0.0, 1e-3);
  EXPECT_NEAR(out.theta, kPi / 2.0, 1e-3);
  EXPECT_NEAR(out.lateral, 0.0, 1e-3);
  EXPECT_NEAR(out.height, kStandoff, 1e-9);  // the standoff is never touched
  EXPECT_TRUE(out.only_vy_wz_nonzero);
}

TEST(IbvsController, CorrectsOffsetAndYawTogether)
{
  IbvsController controller{IbvsControllerParams{}};
  const auto out = simulate(controller, cameraAbove(0.04, kStandoff, 10.0 * kPi / 180.0));
  EXPECT_NEAR(out.rho, 0.0, 1e-3);
  EXPECT_NEAR(out.theta, kPi / 2.0, 1e-3);
  EXPECT_NEAR(out.lateral, 0.0, 1e-3);
  EXPECT_NEAR(out.height, kStandoff, 1e-9);
  EXPECT_TRUE(out.only_vy_wz_nonzero);
}

TEST(IbvsController, KeepsTheMeasuredPolarityInsteadOfTurningHalfARevolution)
{
  // Camera upside down about its optical axis: the panel is now above the edge,
  // so theta is -pi/2 rather than +pi/2.
  const vpHomogeneousMatrix cMo = cameraAbove(-0.03, kStandoff, kPi);
  const auto [rho, theta] = measure(cMo);
  ASSERT_NEAR(theta, -kPi / 2.0, 1e-9);

  IbvsController controller{IbvsControllerParams{}};
  controller.matchPolarity(theta);
  const auto out = simulate(controller, cMo);
  EXPECT_NEAR(out.rho, 0.0, 1e-3);
  EXPECT_NEAR(out.theta, -kPi / 2.0, 1e-3);
  EXPECT_NEAR(out.lateral, 0.0, 1e-3);
  EXPECT_LT(std::abs(out.integrated_wz), 0.05);  // no half turn
}

TEST(IbvsController, NoErrorAndNoMotionAtTheDesiredLine)
{
  IbvsController controller{IbvsControllerParams{}};
  const auto away = controller.computeTwist(-0.15, kPi / 2.0, kStandoff);
  EXPECT_GT(controller.errorNorm(), 0.1);
  EXPECT_NE(away[1], 0.0);

  const auto at_goal = controller.computeTwist(0.0, kPi / 2.0, kStandoff);
  EXPECT_LT(controller.errorNorm(), 1e-12);
  for (const double c : at_goal) {
    EXPECT_NEAR(c, 0.0, 1e-12);
  }
}

TEST(IbvsController, ReturnsZeroTwistForInvalidInput)
{
  IbvsController controller{IbvsControllerParams{}};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & twist : {
      controller.computeTwist(0.1, kPi / 2.0, 0.0),
      controller.computeTwist(0.1, kPi / 2.0, -0.2),
      controller.computeTwist(nan, kPi / 2.0, 0.2),
      controller.computeTwist(0.1, nan, 0.2)})
  {
    for (const double c : twist) {
      EXPECT_EQ(c, 0.0);
    }
  }
}
