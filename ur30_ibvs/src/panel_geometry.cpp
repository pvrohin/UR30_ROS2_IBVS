#include "ur30_ibvs/panel_geometry.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>

namespace ur30_ibvs
{
namespace
{
// Scale v down so its length does not exceed max_norm.
cv::Vec3d limited(const cv::Vec3d & v, double max_norm)
{
  const double n = cv::norm(v);
  return n > max_norm && n > 0.0 ? v * (max_norm / n) : v;
}

cv::Vec3d column(const cv::Matx33d & R, int i)
{
  return {R(0, i), R(1, i), R(2, i)};
}
}  // namespace

cv::Matx33d rotationOf(const cv::Matx44d & T)
{
  return {T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2)};
}

cv::Vec3d translationOf(const cv::Matx44d & T) {return {T(0, 3), T(1, 3), T(2, 3)};}

std::optional<cv::Vec3d> backProjectToPlane(
  const cv::Matx33d & K, const cv::Matx44d & world_T_cam, const cv::Point2f & pixel,
  double plane_z)
{
  const cv::Vec3d ray_cam((pixel.x - K(0, 2)) / K(0, 0), (pixel.y - K(1, 2)) / K(1, 1), 1.0);
  const cv::Vec3d origin = translationOf(world_T_cam);
  const cv::Vec3d ray = rotationOf(world_T_cam) * ray_cam;
  if (std::abs(ray[2]) < 1e-9) {
    return std::nullopt;
  }
  const double s = (plane_z - origin[2]) / ray[2];
  if (s <= 0.0) {
    return std::nullopt;
  }
  return origin + s * ray;
}

std::optional<cv::Point2d> projectToPixel(
  const cv::Matx33d & K, const cv::Matx44d & world_T_cam, const cv::Vec3d & world_point)
{
  const cv::Vec3d p = rotationOf(world_T_cam).t() * (world_point - translationOf(world_T_cam));
  if (p[2] <= 1e-6) {
    return std::nullopt;
  }
  return cv::Point2d(K(0, 0) * p[0] / p[2] + K(0, 2), K(1, 1) * p[1] / p[2] + K(1, 2));
}

std::optional<PanelWorldPose> panelPoseFromCorners(
  const cv::Matx33d & K, const cv::Matx44d & world_T_cam,
  const std::array<cv::Point2f, 4> & corners, double plane_z)
{
  std::array<cv::Vec3d, 4> P;
  for (size_t i = 0; i < 4; ++i) {
    const auto point = backProjectToPlane(K, world_T_cam, corners[i], plane_z);
    if (!point) {
      return std::nullopt;
    }
    P[i] = *point;
  }
  const cv::Vec3d along_x = (P[1] - P[0]) + (P[2] - P[3]);
  PanelWorldPose pose;
  pose.center = 0.25 * (P[0] + P[1] + P[2] + P[3]);
  pose.yaw = std::atan2(along_x[1], along_x[0]);
  pose.length = 0.5 * (cv::norm(P[1] - P[0]) + cv::norm(P[2] - P[3]));
  pose.width = 0.5 * (cv::norm(P[3] - P[0]) + cv::norm(P[2] - P[1]));
  return pose;
}

PanelEdge longEdge(const PanelWorldPose & panel, double panel_width, double sign)
{
  const cv::Vec3d x_axis(std::cos(panel.yaw), std::sin(panel.yaw), 0.0);
  const cv::Vec3d y_axis(-std::sin(panel.yaw), std::cos(panel.yaw), 0.0);
  return {panel.center + (sign * 0.5 * panel_width) * y_axis, x_axis};
}

cv::Matx44d approachTarget(
  const PanelEdge & edge, double standoff, const cv::Matx44d & world_T_cam)
{
  cv::Vec3d x = edge.direction * (1.0 / cv::norm(edge.direction));
  if (x.dot(column(rotationOf(world_T_cam), 0)) < 0.0) {
    x = -x;
  }
  const cv::Vec3d z(0.0, 0.0, -1.0);
  const cv::Vec3d y = z.cross(x);
  const cv::Vec3d p = edge.midpoint + cv::Vec3d(0.0, 0.0, standoff);
  return {x[0], y[0], z[0], p[0],
    x[1], y[1], z[1], p[1],
    x[2], y[2], z[2], p[2],
    0.0, 0.0, 0.0, 1.0};
}

ApproachCommand approachTwist(
  const cv::Matx44d & world_T_cam, const cv::Matx44d & target, double kp_linear,
  double kp_angular, double max_linear, double max_angular)
{
  const cv::Matx33d R = rotationOf(world_T_cam);
  const cv::Vec3d linear_error = translationOf(target) - translationOf(world_T_cam);
  cv::Vec3d angular_error;  // rotation vector taking the current orientation to the target
  cv::Rodrigues(rotationOf(target) * R.t(), angular_error);

  const cv::Vec3d v = R.t() * limited(kp_linear * linear_error, max_linear);
  const cv::Vec3d w = R.t() * limited(kp_angular * angular_error, max_angular);
  return {{v[0], v[1], v[2], w[0], w[1], w[2]}, cv::norm(linear_error), cv::norm(angular_error)};
}

std::array<double, 6> twistInParent(
  const cv::Matx44d & parent_T_child, const std::array<double, 6> & twist_child)
{
  const cv::Matx33d R = rotationOf(parent_T_child);
  const cv::Vec3d t = translationOf(parent_T_child);
  const cv::Vec3d w = R * cv::Vec3d(twist_child[3], twist_child[4], twist_child[5]);
  const cv::Vec3d v = R * cv::Vec3d(twist_child[0], twist_child[1], twist_child[2]) + t.cross(w);
  return {v[0], v[1], v[2], w[0], w[1], w[2]};
}

}  // namespace ur30_ibvs
