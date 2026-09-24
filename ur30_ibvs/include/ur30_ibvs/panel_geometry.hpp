#pragma once

#include <array>
#include <optional>

#include <opencv2/core.hpp>

// Geometry for locating the panel from its image corners and steering the camera
// towards one of its long edges. Pure maths on OpenCV types, no ROS: transforms
// are 4x4 homogeneous matrices, and world_T_cam is the camera optical frame
// expressed in the world (x right, y down, z forward in the image).
namespace ur30_ibvs
{

cv::Matx33d rotationOf(const cv::Matx44d & T);
cv::Vec3d translationOf(const cv::Matx44d & T);

// World point where the ray through `pixel` meets the horizontal plane z = plane_z.
// Empty if the ray is parallel to it or points away from it.
std::optional<cv::Vec3d> backProjectToPlane(
  const cv::Matx33d & K, const cv::Matx44d & world_T_cam, const cv::Point2f & pixel,
  double plane_z);

// Pixel of a world point. Empty if it is behind the camera.
std::optional<cv::Point2d> projectToPixel(
  const cv::Matx33d & K, const cv::Matx44d & world_T_cam, const cv::Vec3d & world_point);

// The panel's top face in the world.
struct PanelWorldPose
{
  cv::Vec3d center;
  double yaw;  // direction of the panel x axis about world z [rad]
  double length;  // measured, along the panel x axis [m]
  double width;  // measured, along the panel y axis [m]
};

// Locate the panel by back-projecting its four image corners onto the plane of the
// table top. This avoids the tilt that PnP reads out of a near head-on view.
// corners follow PanelDetection::corners_px: (-l/2,-w/2), (l/2,-w/2), (l/2,w/2),
// (-l/2,w/2) in the panel frame.
std::optional<PanelWorldPose> panelPoseFromCorners(
  const cv::Matx33d & K, const cv::Matx44d & world_T_cam,
  const std::array<cv::Point2f, 4> & corners, double plane_z);

struct PanelEdge
{
  cv::Vec3d midpoint;
  cv::Vec3d direction;  // unit vector along the edge
};

// The long edge at panel y = sign * panel_width / 2 (sign is +1 or -1).
PanelEdge longEdge(const PanelWorldPose & panel, double panel_width, double sign);

// Camera pose that looks straight down from `standoff` above the edge midpoint with
// the image x axis along the edge. Of the two opposite directions along the edge,
// the one closer to the current image x axis is used, so it never turns half a revolution.
cv::Matx44d approachTarget(
  const PanelEdge & edge, double standoff, const cv::Matx44d & world_T_cam);

struct ApproachCommand
{
  std::array<double, 6> twist;  // camera frame [vx vy vz wx wy wz]
  double position_error;  // [m]
  double rotation_error;  // [rad]
};

// Proportional steering of the camera towards `target`, with the linear and angular
// speeds limited to max_linear and max_angular.
ApproachCommand approachTwist(
  const cv::Matx44d & world_T_cam, const cv::Matx44d & target, double kp_linear,
  double kp_angular, double max_linear, double max_angular);

// Express a twist of a rigid body in another frame on the same body.
// twist_child is [v w] with v the velocity of the child frame's origin and both in
// child axes; parent_T_child is the child frame's pose in the parent. The result
// is the velocity of the PARENT frame's origin plus the angular velocity, in
// parent axes: v_parent = R v_child + t x (R w_child), so a rotation of the camera
// also moves a tool frame that is offset from it.
std::array<double, 6> twistInParent(
  const cv::Matx44d & parent_T_child, const std::array<double, 6> & twist_child);

}  // namespace ur30_ibvs
