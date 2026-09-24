#include "ur30_ibvs/ibvs_controller.hpp"

#include <cmath>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace ur30_ibvs
{
namespace
{
constexpr double kHalfPi = 1.57079632679489661923;
}  // namespace

IbvsController::IbvsController(const IbvsControllerParams & params)
: params_(params), desired_theta_(kHalfPi)
{
  // Plane parameters are refreshed on every call; these only make the features valid.
  s_.buildFrom(0.0, kHalfPi, 0.0, 0.0, 1.0, -1.0);
  sd_.buildFrom(params.desired_rho, desired_theta_, 0.0, 0.0, 1.0, -1.0);

  task_.setServo(vpServo::EYEINHAND_CAMERA);
  task_.setInteractionMatrixType(vpServo::CURRENT);
  task_.setLambda(params.lambda);
  task_.addFeature(s_, sd_);

  // A line constrains two DOF. rho responds to both vy and wx, so use the
  // translation vy, plus wz for theta: a square, well-conditioned 2 x 2 system.
  vpColVector dof(6, 0.0);
  dof[1] = 1.0;  // vy
  dof[5] = 1.0;  // wz
  task_.setCameraDoF(dof);
}

void IbvsController::matchPolarity(double measured_theta)
{
  desired_theta_ = std::copysign(kHalfPi, measured_theta);
}

std::array<double, 6> IbvsController::computeTwist(double rho, double theta, double distance)
{
  std::array<double, 6> twist{};
  if (!(distance > 0.0) || !std::isfinite(rho) || !std::isfinite(theta)) {
    return twist;
  }

  // The edge lies in a plane roughly perpendicular to the optical axis at the
  // given distance: Z = distance, i.e. 0 X + 0 Y + 1 Z - distance = 0.
  s_.buildFrom(rho, theta, 0.0, 0.0, 1.0, -distance);
  sd_.buildFrom(params_.desired_rho, desired_theta_, 0.0, 0.0, 1.0, -distance);

  try {
    const vpColVector v = task_.computeControlLaw();
    error_norm_ = task_.getError().frobeniusNorm();
    for (unsigned int i = 0; i < 6; ++i) {
      twist[i] = v[i];
    }
  } catch (const vpException &) {
    return std::array<double, 6>{};
  }
  return twist;
}

}  // namespace ur30_ibvs
