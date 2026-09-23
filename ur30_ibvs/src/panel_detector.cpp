#include "ur30_ibvs/panel_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace ur30_ibvs
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
// Boundary pixels farther than this from a rough side are not used for its line fit.
constexpr double kEdgeBandPx = 3.0;
// Ignore the ends of each side, where the corner rounds off.
constexpr double kSideMargin = 0.1;
constexpr size_t kMinSidePoints = 20;

double angularDistance(double a, double b)
{
  double d = std::fmod(a - b, 2.0 * kPi);
  if (d > kPi) {
    d -= 2.0 * kPi;
  } else if (d < -kPi) {
    d += 2.0 * kPi;
  }
  return std::abs(d);
}

// Intersection of the lines p1 + s*d1 and p2 + u*d2.
std::optional<cv::Point2f> intersect(const cv::Vec4f & l1, const cv::Vec4f & l2)
{
  const cv::Point2f d1(l1[0], l1[1]);
  const cv::Point2f d2(l2[0], l2[1]);
  const float denom = d1.cross(d2);
  if (std::abs(denom) < 1e-6f) {
    return std::nullopt;
  }
  const cv::Point2f p1(l1[2], l1[3]);
  const cv::Point2f p2(l2[2], l2[3]);
  const float s = (p2 - p1).cross(d2) / denom;
  return p1 + s * d1;
}

// Sub-pixel corners of the four-sided blob: approximate its hull to four
// vertices, fit a line to the boundary pixels of each side, and intersect
// neighbouring lines. Unlike a bounding rectangle this also handles a panel that
// is turned or seen in perspective (a trapezoid).
std::optional<std::array<cv::Point2f, 4>> fitQuadrilateral(const std::vector<cv::Point> & contour)
{
  std::vector<cv::Point> hull;
  cv::convexHull(contour, hull);
  const double perimeter = cv::arcLength(hull, true);
  std::vector<cv::Point> approx;
  for (double epsilon = 0.01; epsilon <= 0.10; epsilon += 0.005) {
    cv::approxPolyDP(hull, approx, epsilon * perimeter, true);
    if (approx.size() <= 4) {
      break;
    }
  }
  if (approx.size() != 4) {
    return std::nullopt;
  }

  cv::Point2f centroid(0.f, 0.f);
  for (const auto & v : approx) {
    centroid += cv::Point2f(v) * 0.25f;
  }

  // Line i runs along the side from approx[i] to approx[i + 1].
  std::array<cv::Vec4f, 4> lines;
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f a(approx[i]);
    const cv::Point2f b(approx[(i + 1) % 4]);
    const cv::Point2f ab = b - a;
    const double length = std::sqrt(static_cast<double>(ab.dot(ab)));
    if (length < 1.0) {
      return std::nullopt;
    }

    std::vector<cv::Point2f> side;
    for (const auto & p : contour) {
      const cv::Point2f q(p);
      const double t = (q - a).dot(ab) / (length * length);
      if (t < kSideMargin || t > 1.0 - kSideMargin) {
        continue;
      }
      if (std::abs((q - a).cross(ab)) / length < kEdgeBandPx) {
        side.push_back(q);
      }
    }
    if (side.size() < kMinSidePoints) {
      return std::nullopt;
    }

    cv::Vec4f line;
    cv::fitLine(side, line, cv::DIST_L2, 0, 0.01, 0.01);
    // Boundary pixel centres lie half a pixel inside the true edge, so move the
    // line outwards. The bias would otherwise shrink both sides of the panel by
    // about one pixel and read as a tilt.
    cv::Point2f normal(-line[1], line[0]);
    if (normal.dot(cv::Point2f(line[2], line[3]) - centroid) < 0.f) {
      normal = -normal;
    }
    line[2] += 0.5f * normal.x;
    line[3] += 0.5f * normal.y;
    lines[i] = line;
  }

  std::array<cv::Point2f, 4> corners;
  for (int i = 0; i < 4; ++i) {
    // Vertex i joins side i - 1 and side i.
    const auto corner = intersect(lines[(i + 3) % 4], lines[i]);
    if (!corner) {
      return std::nullopt;
    }
    corners[i] = *corner;
  }
  return corners;
}
}  // namespace

PanelDetector::PanelDetector(const PanelDetectorParams & params) : params_(params)
{
  const auto hl = static_cast<float>(params.panel_length / 2.0);
  const auto hw = static_cast<float>(params.panel_width / 2.0);
  object_corners_ = {{-hl, -hw, 0.f}, {hl, -hw, 0.f}, {hl, hw, 0.f}, {-hl, hw, 0.f}};
}

std::optional<PanelDetection> PanelDetector::detect(
  const cv::Mat & bgr, const cv::Matx33d & K) const
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return std::nullopt;
  }

  // The panel (and its armrest and speaker, which sit inside its outline) is
  // darker than the table, so one threshold merges them into a single blob.
  std::vector<cv::Mat> channels;
  cv::split(bgr, channels);
  cv::Mat value;
  cv::max(channels[0], channels[1], value);
  cv::max(value, channels[2], value);
  cv::Mat mask;
  cv::threshold(value, mask, params_.dark_threshold, 255, cv::THRESH_BINARY_INV);
  // Closing only fills small gaps. An opening would chamfer the corners of a
  // panel that is not aligned with the image axes.
  cv::morphologyEx(
    mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.empty()) {
    return std::nullopt;
  }
  const auto largest = std::max_element(
    contours.begin(), contours.end(), [](const auto & a, const auto & b) {
      return cv::contourArea(a) < cv::contourArea(b);
    });

  const double area = cv::contourArea(*largest);
  const double fraction = area / (static_cast<double>(bgr.rows) * bgr.cols);
  if (fraction < params_.min_area_fraction || fraction > params_.max_area_fraction) {
    return std::nullopt;
  }

  const auto quad = fitQuadrilateral(*largest);
  if (!quad) {
    return std::nullopt;
  }
  const std::vector<cv::Point2f> quad_points(quad->begin(), quad->end());
  const double quad_area = cv::contourArea(quad_points);
  if (quad_area <= 0.0 || area / quad_area < params_.min_rectangularity) {
    return std::nullopt;
  }

  // Compare opposite side lengths to the panel aspect ratio.
  const auto side = [&](int i) {return cv::norm((*quad)[(i + 1) % 4] - (*quad)[i]);};
  const double extent_a = 0.5 * (side(0) + side(2));
  const double extent_b = 0.5 * (side(1) + side(3));
  const double long_side = std::max(extent_a, extent_b);
  const double short_side = std::min(extent_a, extent_b);
  const double expected_ratio = params_.panel_length / params_.panel_width;
  if (short_side <= 0.0 ||
    std::abs(long_side / short_side / expected_ratio - 1.0) > params_.aspect_tolerance)
  {
    return std::nullopt;
  }

  // Which image corner is which panel corner is unknown, so try every cyclic
  // shift in both directions. Wrong assignments fail the reprojection or the
  // facing-the-camera test. The two 180-degree-related shifts fit equally
  // well, and the expected x-axis direction picks between them.
  const double expected_angle = params_.expected_x_axis_angle_deg * kPi / 180.0;
  std::optional<PanelDetection> best;
  double best_cost = std::numeric_limits<double>::infinity();

  const auto consider = [&](const PanelDetection & c) {
      std::vector<cv::Point2f> projected;
      cv::projectPoints(
        std::vector<cv::Point3f>{object_corners_[0], object_corners_[1]}, c.rvec, c.tvec, K,
        cv::noArray(), projected);
      const double x_axis_angle =
        std::atan2(projected[1].y - projected[0].y, projected[1].x - projected[0].x);
      const double cost = angularDistance(x_axis_angle, expected_angle);
      if (cost < best_cost) {
        best_cost = cost;
        best = c;
      }
    };

  for (const int direction : {1, -1}) {
    for (int shift = 0; shift < 4; ++shift) {
      std::vector<cv::Point2f> image_points(4);
      for (int i = 0; i < 4; ++i) {
        image_points[i] = (*quad)[((direction * i + shift) % 4 + 4) % 4];
      }

      std::vector<cv::Mat> rvecs, tvecs;
      const int n = cv::solvePnPGeneric(
        object_corners_, image_points, K, cv::noArray(), rvecs, tvecs, false,
        cv::SOLVEPNP_IPPE);

      // IPPE returns two solutions per assignment (planar tilt ambiguity);
      // keep the better-fitting valid one before comparing assignments.
      std::optional<PanelDetection> candidate;
      for (int k = 0; k < n; ++k) {
        // IPPE's analytic solution can leave a large residual for a near
        // head-on view (perspective is traded away to explain a tiny
        // foreshortening), so refine it by minimising the reprojection error.
        cv::Mat refined_rvec = rvecs[k].clone();
        cv::Mat refined_tvec = tvecs[k].clone();
        cv::solvePnPRefineLM(
          object_corners_, image_points, K, cv::noArray(), refined_rvec, refined_tvec);
        const cv::Vec3d rvec(refined_rvec);
        const cv::Vec3d tvec(refined_tvec);
        // solvePnPGeneric's own reprojection error output is unreliable for
        // IPPE in OpenCV 4.6 (it can come back negative), so compute the RMS
        // error here.
        std::vector<cv::Point2f> reprojected;
        cv::projectPoints(object_corners_, rvec, tvec, K, cv::noArray(), reprojected);
        double squared = 0.0;
        for (int i = 0; i < 4; ++i) {
          const double d = cv::norm(reprojected[i] - image_points[i]);
          squared += d * d;
        }
        const double error = std::sqrt(squared / 4.0);
        if (tvec[2] <= 0.0 || error > params_.max_reprojection_px) {
          continue;
        }
        cv::Matx33d R;
        cv::Rodrigues(rvec, R);
        const cv::Vec3d normal(R(0, 2), R(1, 2), R(2, 2));
        if (normal.dot(tvec) >= 0.0) {
          continue;  // top face turned away from the camera
        }
        if (!candidate || error < candidate->reprojection_error_px) {
          candidate = PanelDetection{
            rvec, tvec, error,
            {image_points[0], image_points[1], image_points[2], image_points[3]}};
        }
      }
      if (!candidate) {
        continue;
      }
      consider(*candidate);

      // The half-turn twin fits identically, but PnP is unreliable at solving
      // that degenerate on-axis case, so build it: the same pose turned 180
      // degrees about the panel z axis, with the corner labels shifted by two.
      cv::Matx33d R;
      cv::Rodrigues(candidate->rvec, R);
      PanelDetection twin = *candidate;
      cv::Rodrigues(R * cv::Matx33d(-1, 0, 0, 0, -1, 0, 0, 0, 1), twin.rvec);
      std::rotate(twin.corners_px.begin(), twin.corners_px.begin() + 2, twin.corners_px.end());
      consider(twin);
    }
  }
  return best;
}

}  // namespace ur30_ibvs
