#pragma once

#include <optional>

#include <opencv2/core.hpp>

namespace ur30_ibvs
{

struct CornerFinderParams
{
  // Pixels whose brightest channel is below this count as part (same meaning as the
  // panel detector's dark_threshold).
  int dark_threshold = 150;
  // The strip that is read runs parallel to the tracked edge, this far on the part side
  // of it [px]. Close to the edge, so that it ends at the corner, but not on the edge
  // itself, where the pixels are mixed.
  double strip_offset_px = 10.0;
  // The part has ended when the strip has been bright this many pixels in a row.
  int bright_run_px = 6;
  // ... but only after the strip has been dark this many pixels (in total): the part has
  // to be seen before it can end.
  int dark_run_px = 12;
};

// Finds where a straight edge of the part ends at a corner, in an image in which the
// edge is (nearly) horizontal and the camera moves towards +x. Works on the image, not
// on the tracker's points, so it does not depend on how ViSP drops sites near the end.
class CornerFinder
{
public:
  explicit CornerFinder(const CornerFinderParams & params);

  // bgr: 8-bit 3-channel image. rho, theta: the tracked line in ViSP's pixel convention
  // (i cos(theta) + j sin(theta) = rho, i = row, j = column; EdgeTracker::rhoTheta()).
  // from_col: first column to read; it must lie on the edge, behind the corner.
  // Returns the column at which the part ends, or nothing if the strip stays dark to the
  // right image border (the corner is not in view yet) or the line is not usable.
  std::optional<double> find(
    const cv::Mat & bgr, double rho, double theta, double from_col) const;

private:
  CornerFinderParams params_;
};

}  // namespace ur30_ibvs
