#pragma once

#include <array>

#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/vs/vpServo.h>

namespace ur30_ibvs
{

struct IbvsControllerParams
{
  double lambda = 0.6;  // servo gain [1/s]
  double desired_rho = 0.0;  // desired line offset, normalised image units (0 = through the centre)
};

// Eye-in-hand image-based servoing on one straight edge, in the optical frame
// (x right, y down, z forward).
//
// A line feature (rho, theta) gives two measurements, so only two camera DOF are
// controlled: the translation across the edge (vy) and the rotation about the
// optical axis (wz). Together they centre the edge and keep it parallel to the
// image x axis. Everything else is zero here: the standoff (vz) and any motion
// along the edge (vx) cannot be observed from one line and are commanded by the
// caller, and wx, wy are held at zero.
class IbvsController
{
public:
  explicit IbvsController(const IbvsControllerParams & params);

  // vpServo holds pointers to this object's features.
  IbvsController(const IbvsController &) = delete;
  IbvsController & operator=(const IbvsController &) = delete;

  // A horizontal edge has theta = +pi/2 when the dark side is below it in the
  // image and -pi/2 when it is above. Take the desired sign from a measurement
  // so the controller never tries to turn the camera half a revolution.
  void matchPolarity(double measured_theta);

  // Camera-frame twist [vx vy vz wx wy wz] (m/s, rad/s) that moves the measured
  // line (normalised image coordinates) towards the desired one. distance is the
  // camera's distance to the plane containing the edge [m], which sets the scale
  // of the interaction matrix. Returns zeros for a non-positive distance or a
  // non-finite measurement.
  std::array<double, 6> computeTwist(double rho, double theta, double distance);

  // Norm of the feature error at the last computeTwist().
  double errorNorm() const {return error_norm_;}

private:
  IbvsControllerParams params_;
  double desired_theta_;
  double error_norm_ = 0.0;
  VISP_NAMESPACE_ADDRESSING vpFeatureLine s_;
  VISP_NAMESPACE_ADDRESSING vpFeatureLine sd_;
  VISP_NAMESPACE_ADDRESSING vpServo task_;
};

}  // namespace ur30_ibvs
