#include "ur30_ibvs/servo_supervisor.hpp"

namespace ur30_ibvs
{

ServoSupervisor::ServoSupervisor(double image_timeout_sec, double edge_lost_timeout_sec)
: image_timeout_(image_timeout_sec), edge_timeout_(edge_lost_timeout_sec)
{
}

void ServoSupervisor::start(double now)
{
  last_image_ = now;
  missing_since_ = now;
}

void ServoSupervisor::onImage(double now) {last_image_ = now;}

void ServoSupervisor::onEdgeLocked() {missing_since_.reset();}

void ServoSupervisor::onEdgeMissing(double now)
{
  if (!missing_since_) {
    missing_since_ = now;
  }
}

SupervisorVerdict ServoSupervisor::check(double now) const
{
  if (now - last_image_ > image_timeout_) {
    return SupervisorVerdict::CAMERA_LOST;
  }
  if (missing_since_ && now - *missing_since_ > edge_timeout_) {
    return SupervisorVerdict::EDGE_LOST;
  }
  return SupervisorVerdict::OK;
}

double ServoSupervisor::secondsSinceImage(double now) const {return now - last_image_;}

double ServoSupervisor::secondsEdgeMissing(double now) const
{
  return missing_since_ ? now - *missing_since_ : 0.0;
}

}  // namespace ur30_ibvs
