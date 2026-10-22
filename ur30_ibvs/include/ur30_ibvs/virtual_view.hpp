#pragma once

#include <array>

#include <opencv2/core.hpp>

// A "virtual camera": the real camera turned about its optical axis by a multiple of
// 90 degrees. Seen through it, an image edge that is vertical in the real image looks
// horizontal, so the edge controller (written for a horizontal edge) can be used on
// every side of a square. Pure maths on OpenCV types, no ROS.
//
// quarter_turns counts 90 degree CLOCKWISE turns of the image (cv::ROTATE_90_CLOCKWISE),
// modulo 4. Pixels use OpenCV's convention (pixel centres at integer coordinates).
namespace ur30_ibvs
{

cv::Mat rotateImage(const cv::Mat & image, int quarter_turns);

cv::Size rotatedSize(const cv::Size & size, int quarter_turns);

// Pixel of the rotated image that shows the same scene point as `pixel` of the original.
cv::Point2d rotatePixel(const cv::Point2d & pixel, const cv::Size & size, int quarter_turns);

// Pinhole intrinsics of the rotated image. `size` is the size of the original image.
cv::Matx33d rotateIntrinsics(const cv::Matx33d & K, const cv::Size & size, int quarter_turns);

// A twist [vx vy vz wx wy wz] of the virtual camera, expressed in its own axes, as the
// same motion in the axes of the real camera.
std::array<double, 6> twistFromVirtual(const std::array<double, 6> & twist, int quarter_turns);

// The number of quarter turns that makes the virtual +x axis point along (dx, dy), a
// direction in the real image plane (x right, y down). The nearest of the four is used.
int quarterTurnsForDirection(double dx, double dy);

}  // namespace ur30_ibvs
