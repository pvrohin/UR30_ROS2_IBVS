#pragma once

#include <array>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace ur30_ibvs
{

struct PanelDetectorParams
{
  // Real panel outline. The pose frame is the centre of the top face: x along
  // the length, y along the width, z = 0 on the face and pointing at the camera.
  double panel_length = 0.9;
  double panel_width = 0.55;

  // Pixels whose brightest channel is below this count as panel.
  int dark_threshold = 90;
  double min_area_fraction = 0.35;  // panel blob area / image area
  double max_area_fraction = 0.85;
  double min_rectangularity = 0.85;  // blob area / its min-area rectangle
  double aspect_tolerance = 0.2;     // relative error on length / width
  double max_reprojection_px = 3.0;

  // A rectangle looks identical after a 180 degree in-plane turn, so the
  // outline alone cannot say which long edge is which. Pick the candidate whose
  // panel x axis points closest to this image direction (degrees, 0 = image
  // right, positive = clockwise on screen).
  double expected_x_axis_angle_deg = 0.0;
};

struct PanelDetection
{
  // Pose of the panel frame in the camera frame (cv::Rodrigues convention).
  // Depth and in-plane position are accurate, but for a near head-on view the
  // tilt is poorly observable (foreshortening is second order: 0.3 px of corner
  // noise reads as ~2.7 deg at the survey pose). Callers that know the panel
  // lies on a known plane should prefer corners_px.
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  double reprojection_error_px = 0.0;

  // Detected image corners in the order of the panel corners
  // (-l/2,-w/2), (l/2,-w/2), (l/2,w/2), (-l/2,w/2).
  std::array<cv::Point2f, 4> corners_px;
};

class PanelDetector
{
public:
  explicit PanelDetector(const PanelDetectorParams & params);

  // bgr: 8-bit 3-channel image. K: pinhole intrinsics, distortion ignored.
  std::optional<PanelDetection> detect(const cv::Mat & bgr, const cv::Matx33d & K) const;

private:
  PanelDetectorParams params_;
  std::vector<cv::Point3f> object_corners_;
};

}  // namespace ur30_ibvs
