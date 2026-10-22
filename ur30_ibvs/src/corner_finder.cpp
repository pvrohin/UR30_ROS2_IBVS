#include "ur30_ibvs/corner_finder.hpp"

#include <algorithm>
#include <cmath>

namespace ur30_ibvs
{
namespace
{
// Brightest channel of the pixel, averaged over three rows around `row`. Negative when
// the row is outside the image.
double stripValue(const cv::Mat & bgr, int col, double row)
{
  const int r = static_cast<int>(std::lround(row));
  if (col < 0 || col >= bgr.cols || r < 1 || r >= bgr.rows - 1) {
    return -1.0;
  }
  double sum = 0.0;
  for (int dr = -1; dr <= 1; ++dr) {
    const cv::Vec3b & p = bgr.at<cv::Vec3b>(r + dr, col);
    sum += std::max({p[0], p[1], p[2]});
  }
  return sum / 3.0;
}
}  // namespace

CornerFinder::CornerFinder(const CornerFinderParams & params) : params_(params) {}

std::optional<double> CornerFinder::find(
  const cv::Mat & bgr, double rho, double theta, double from_col) const
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return std::nullopt;
  }
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  if (std::abs(c) < 0.5) {
    return std::nullopt;  // not a roughly horizontal line
  }
  const auto line_row = [&](double col) {return (rho - col * s) / c;};

  const int start = std::max(0, static_cast<int>(std::lround(from_col)));
  // Which side of the edge is the part: the one that reads dark just beyond from_col.
  const int probe_end = std::min(bgr.cols - 1, start + 40);
  double above = 0.0;
  double below = 0.0;
  int n = 0;
  for (int col = start; col <= probe_end; ++col, ++n) {
    above += stripValue(bgr, col, line_row(col) - params_.strip_offset_px);
    below += stripValue(bgr, col, line_row(col) + params_.strip_offset_px);
  }
  if (n == 0) {
    return std::nullopt;
  }
  above /= n;
  below /= n;
  // The part is on the darker side. Beside a corner both sides can be bright for part of
  // the probe window, so a side that is clearly darker than the other also counts.
  constexpr double kClearlyDarker = 25.0;
  if (above < 0.0 && below < 0.0) {
    return std::nullopt;
  }
  const double dark_side = above < 0.0 ? below : (below < 0.0 ? above : std::min(above, below));
  const double other = above < 0.0 || below < 0.0 ? 255.0 : std::max(above, below);
  if (!(dark_side < params_.dark_threshold || other - dark_side >= kClearlyDarker)) {
    return std::nullopt;  // no part beside the edge here
  }
  const double side = (above >= 0.0 && (below < 0.0 || above <= below)) ? -1.0 : 1.0;

  // The part must be seen before it can end: a bright stretch at the start of the strip
  // is the table beyond the previous corner (or the part has just moved there), not a
  // corner ahead.
  int dark = 0;
  int bright = 0;
  for (int col = start; col < bgr.cols; ++col) {
    const double v = stripValue(bgr, col, line_row(col) + side * params_.strip_offset_px);
    if (v < 0.0) {
      return std::nullopt;  // the strip left the image before the part ended
    }
    if (v >= params_.dark_threshold) {
      if (dark >= params_.dark_run_px && ++bright >= params_.bright_run_px) {
        return static_cast<double>(col - bright + 1);
      }
    } else {
      ++dark;
      bright = 0;
    }
  }
  return std::nullopt;
}

}  // namespace ur30_ibvs
