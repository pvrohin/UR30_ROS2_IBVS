#pragma once

#include <optional>
#include <vector>

namespace ur30_ibvs
{

struct SurveyLimits
{
  double min_duration_sec = 6.0;  // never faster than this
  double max_joint_speed = 0.5;  // [rad/s] average, on the joint that moves the most
  double max_joint_distance = 6.3;  // [rad] refuse a survey move larger than this
};

// Largest change of any joint. Empty if the vectors differ in size or are empty.
std::optional<double> largestJointMove(
  const std::vector<double> & from, const std::vector<double> & to);

// How long a move between two joint positions should take, so that its speed does not depend
// on where the arm happens to start. Empty if the move must not be made: it is too large, or
// the inputs are not valid joint positions.
std::optional<double> surveyDuration(
  const std::vector<double> & from, const std::vector<double> & to, const SurveyLimits & limits);

}  // namespace ur30_ibvs
