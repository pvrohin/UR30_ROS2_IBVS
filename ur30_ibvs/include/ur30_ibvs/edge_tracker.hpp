#pragma once

#include <opencv2/core.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureLine.h>

namespace ur30_ibvs
{

struct EdgeTrackerParams
{
  unsigned int range = 10;  // search range either side of the line [px]
  double sample_step = 5.0;  // distance between tracked points along the line [px]
  double threshold = 20.0;  // minimum luminance contrast of an edge, in [0, 255]
  // Tracking counts as lost once fewer than this fraction of the points found
  // at initialisation are still tracked.
  double min_tracked_fraction = 0.5;
  // init() moves each seed point along the edge normal to the strongest edge
  // within this distance [px]. ViSP's own initialisation only works when the
  // seed lies on the edge (a 4 px error already fails), whereas a seed
  // projected from an estimated pose is typically tens of pixels off.
  double snap_radius = 50.0;
};

// Tracks a single straight image edge with ViSP's moving-edges line tracker.
class EdgeTracker
{
public:
  explicit EdgeTracker(const EdgeTrackerParams & params);

  // vpMeLine keeps a raw pointer to the vpMe settings owned by this object, so
  // a copy or move would leave it pointing at the old one.
  EdgeTracker(const EdgeTracker &) = delete;
  EdgeTracker & operator=(const EdgeTracker &) = delete;

  // Start tracking the edge near the segment p1-p2 (pixels x, y; near opposite
  // ends of the edge). Each point is first moved along the segment's normal to the
  // strongest edge within snap_radius, so the seed need not be exact. image is
  // 8-bit, 1 or 3 channels. Returns false if no edge could be found.
  bool init(const cv::Mat & image, const cv::Point2d & p1, const cv::Point2d & p2);

  // Advance to the next image. Returns false once the edge is lost.
  bool track(const cv::Mat & image);

  bool isTracking() const {return tracking_;}
  int trackedPoints() const;

  // Line feature (rho, theta) in normalised image coordinates for vpServo.
  void feature(
    const VISP_NAMESPACE_ADDRESSING vpCameraParameters & cam,
    VISP_NAMESPACE_ADDRESSING vpFeatureLine & s) const;

  // Line parameters in ViSP's pixel convention: theta is measured from the
  // vertical image axis. Mostly useful for debugging.
  cv::Vec2d rhoTheta() const;

private:
  EdgeTrackerParams params_;
  VISP_NAMESPACE_ADDRESSING vpMe me_;
  VISP_NAMESPACE_ADDRESSING vpMeLine line_;
  bool tracking_ = false;
  int initial_points_ = 0;
};

}  // namespace ur30_ibvs
