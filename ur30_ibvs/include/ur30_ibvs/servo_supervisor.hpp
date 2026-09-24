#pragma once

#include <optional>

namespace ur30_ibvs
{

enum class SupervisorVerdict
{
  OK,
  CAMERA_LOST,  // no image for longer than the image timeout
  EDGE_LOST     // the edge has been missing for longer than the edge timeout
};

// Decides, while servoing on an edge, when to give up. Time is passed in (seconds
// on any monotonic clock), so it has no ROS dependency and is easy to test.
//
// The edge tolerance is a duration, not a count of frames: a frame count is a
// proxy for time that changes with the camera rate and with dropped frames.
class ServoSupervisor
{
public:
  ServoSupervisor(double image_timeout_sec, double edge_lost_timeout_sec);

  // Call on entering SERVO. The edge counts as missing until it is first locked.
  void start(double now);

  void onImage(double now);
  void onEdgeLocked();  // the edge was found / tracked on this frame
  void onEdgeMissing(double now);  // it was lost, or could not be found

  // Camera loss takes precedence: with no images the edge state is unknown.
  SupervisorVerdict check(double now) const;

  double secondsSinceImage(double now) const;
  double secondsEdgeMissing(double now) const;  // 0 while the edge is locked

private:
  double image_timeout_;
  double edge_timeout_;
  double last_image_ = 0.0;
  std::optional<double> missing_since_;
};

}  // namespace ur30_ibvs
