#include "ur30_ibvs/edge_tracker.hpp"

#include <cmath>
#include <optional>

#include <opencv2/imgproc.hpp>

#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace ur30_ibvs
{
namespace
{
// Half-width of the strip along the edge that the snap profile is averaged over.
constexpr int kSnapHalfWidth = 8;
// The two snapped points must still describe roughly the seeded line.
constexpr double kMaxSnapAngleRad = 10.0 * 3.14159265358979323846 / 180.0;

// False if the image is not 8-bit with 1 or 3 channels.
bool toGray(const cv::Mat & image, cv::Mat & gray)
{
  if (image.empty() || image.depth() != CV_8U) {
    return false;
  }
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 1) {
    gray = image;
  } else {
    return false;
  }
  return true;
}

bool toViSP(const cv::Mat & image, vpImage<unsigned char> & out)
{
  cv::Mat gray;
  if (!toGray(image, gray)) {
    return false;
  }
  vpImageConvert::convert(gray, out);
  return true;
}

// Move p along the normal of direction d to the strongest intensity edge within
// radius. The profile is averaged over a strip along the edge to suppress noise.
std::optional<cv::Point2d> snapToEdge(
  const cv::Mat & gray32, const cv::Point2d & p, const cv::Point2d & d, double radius,
  double min_contrast)
{
  const cv::Point2d n(-d.y, d.x);
  const int R = static_cast<int>(std::lround(radius));
  const int cols = 2 * R + 1;
  const int rows = 2 * kSnapHalfWidth + 1;

  // Patch pixel (u, v) samples the image at p + n * (u - R) + d * (v - half).
  const cv::Mat M = (cv::Mat_<double>(2, 3) <<
    n.x, d.x, p.x - n.x * R - d.x * kSnapHalfWidth,
    n.y, d.y, p.y - n.y * R - d.y * kSnapHalfWidth);
  cv::Mat patch;
  cv::warpAffine(
    gray32, patch, M, cv::Size(cols, rows), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
    cv::BORDER_REPLICATE);
  cv::Mat profile;
  cv::reduce(patch, profile, 0, cv::REDUCE_AVG);

  double strongest = 0.0;
  int best_u = -1;
  for (int u = 1; u < cols - 1; ++u) {
    const double gradient = std::abs(profile.at<float>(0, u + 1) - profile.at<float>(0, u - 1));
    if (gradient > strongest) {
      strongest = gradient;
      best_u = u;
    }
  }
  if (best_u < 0 || strongest < min_contrast) {
    return std::nullopt;
  }
  return p + n * static_cast<double>(best_u - R);
}
}  // namespace

EdgeTracker::EdgeTracker(const EdgeTrackerParams & params) : params_(params)
{
  // NORMALIZED_THRESHOLD makes threshold a luminance contrast in [0, 255]; the
  // default (OLD_THRESHOLD) expects values around 10000. The negative margin
  // ratio and minimum switch off the automatic threshold.
  me_.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
  me_.setThreshold(params.threshold);
  me_.setThresholdMarginRatio(-1.);
  me_.setMinThreshold(-1.);
  me_.setRange(params.range);
  me_.setSampleStep(params.sample_step);
  me_.setMu1(0.5);
  me_.setMu2(0.5);
  me_.initMask();
  line_.setMe(&me_);
}

bool EdgeTracker::init(const cv::Mat & image, const cv::Point2d & p1, const cv::Point2d & p2)
{
  tracking_ = false;
  cv::Mat gray;
  if (!toGray(image, gray)) {
    return false;
  }
  const cv::Point2d seed = p2 - p1;
  const double seed_length = std::hypot(seed.x, seed.y);
  if (seed_length < 1.0) {
    return false;
  }
  const cv::Point2d direction = seed / seed_length;

  cv::Mat gray32;
  gray.convertTo(gray32, CV_32F);
  const auto q1 = snapToEdge(gray32, p1, direction, params_.snap_radius, params_.threshold);
  const auto q2 = snapToEdge(gray32, p2, direction, params_.snap_radius, params_.threshold);
  if (!q1 || !q2) {
    return false;
  }
  const cv::Point2d snapped = *q2 - *q1;
  const double snapped_length = std::hypot(snapped.x, snapped.y);
  if (snapped_length < 1.0 ||
    std::acos(std::min(1.0, snapped.dot(direction) / snapped_length)) > kMaxSnapAngleRad)
  {
    return false;
  }

  vpImage<unsigned char> I;
  vpImageConvert::convert(gray, I);
  try {
    line_.initTracking(I, vpImagePoint(q1->y, q1->x), vpImagePoint(q2->y, q2->x));
    // initTracking only samples points along the segment: rho and theta stay at
    // zero until the first track(), so run it now on the same image.
    line_.track(I);
  } catch (const vpException &) {
    return false;
  }
  initial_points_ = line_.getNbPoints();
  tracking_ = initial_points_ > 0;
  return tracking_;
}

bool EdgeTracker::track(const cv::Mat & image)
{
  if (!tracking_) {
    return false;
  }
  vpImage<unsigned char> I;
  if (!toViSP(image, I)) {
    tracking_ = false;
    return false;
  }
  try {
    line_.track(I);
  } catch (const vpException &) {
    tracking_ = false;
    return false;
  }
  if (line_.getNbPoints() < params_.min_tracked_fraction * initial_points_) {
    tracking_ = false;
  }
  return tracking_;
}

int EdgeTracker::trackedPoints() const {return line_.getNbPoints();}

std::vector<cv::Point2f> EdgeTracker::sitePixels() const
{
  std::vector<cv::Point2f> pixels;
  for (const vpMeSite & site : line_.getMeList()) {
    if (site.getState() == vpMeSite::NO_SUPPRESSION) {
      pixels.emplace_back(static_cast<float>(site.get_jfloat()), static_cast<float>(site.get_ifloat()));
    }
  }
  return pixels;
}

void EdgeTracker::feature(const vpCameraParameters & cam, vpFeatureLine & s) const
{
  vpFeatureBuilder::create(s, cam, line_);
}

cv::Vec2d EdgeTracker::rhoTheta() const {return {line_.getRho(), line_.getTheta()};}

}  // namespace ur30_ibvs
