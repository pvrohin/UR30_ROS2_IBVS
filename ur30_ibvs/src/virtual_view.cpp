#include "ur30_ibvs/virtual_view.hpp"

#include <cmath>

#include <opencv2/core.hpp>

namespace ur30_ibvs
{
namespace
{
int wrap(int quarter_turns) {return ((quarter_turns % 4) + 4) % 4;}
}  // namespace

cv::Mat rotateImage(const cv::Mat & image, int quarter_turns)
{
  cv::Mat out;
  switch (wrap(quarter_turns)) {
    case 1: cv::rotate(image, out, cv::ROTATE_90_CLOCKWISE); break;
    case 2: cv::rotate(image, out, cv::ROTATE_180); break;
    case 3: cv::rotate(image, out, cv::ROTATE_90_COUNTERCLOCKWISE); break;
    default: out = image; break;
  }
  return out;
}

cv::Size rotatedSize(const cv::Size & size, int quarter_turns)
{
  return wrap(quarter_turns) % 2 == 0 ? size : cv::Size(size.height, size.width);
}

cv::Point2d rotatePixel(const cv::Point2d & pixel, const cv::Size & size, int quarter_turns)
{
  cv::Point2d p = pixel;
  cv::Size s = size;
  for (int i = 0; i < wrap(quarter_turns); ++i) {
    // One clockwise quarter turn: (u, v) -> (h - 1 - v, u), the size becomes (h, w).
    p = cv::Point2d(s.height - 1 - p.y, p.x);
    s = cv::Size(s.height, s.width);
  }
  return p;
}

cv::Matx33d rotateIntrinsics(const cv::Matx33d & K, const cv::Size & size, int quarter_turns)
{
  cv::Matx33d out = K;
  cv::Size s = size;
  for (int i = 0; i < wrap(quarter_turns); ++i) {
    out = cv::Matx33d(
      out(1, 1), 0.0, s.height - 1 - out(1, 2),
      0.0, out(0, 0), out(0, 2),
      0.0, 0.0, 1.0);
    s = cv::Size(s.height, s.width);
  }
  return out;
}

std::array<double, 6> twistFromVirtual(const std::array<double, 6> & twist, int quarter_turns)
{
  std::array<double, 6> out = twist;
  for (int i = 0; i < wrap(quarter_turns); ++i) {
    // The virtual x axis is the real -y axis and the virtual y axis the real +x axis, so
    // a vector (a, b) in virtual axes is (b, -a) in real axes. z is unchanged.
    const auto turn = [&out](int at) {
        const double a = out[at];
        const double b = out[at + 1];
        out[at] = b;
        out[at + 1] = -a;
      };
    turn(0);
    turn(3);
  }
  return out;
}

int quarterTurnsForDirection(double dx, double dy)
{
  // Angle of the direction on the screen, counter-clockwise from image right (y is down).
  const double angle = std::atan2(-dy, dx);
  return wrap(static_cast<int>(std::lround(angle / (M_PI / 2.0))));
}

}  // namespace ur30_ibvs
