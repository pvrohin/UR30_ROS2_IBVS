#include "ur30_ibvs/survey_motion.hpp"

#include <algorithm>
#include <cmath>

namespace ur30_ibvs
{

std::optional<double> largestJointMove(
  const std::vector<double> & from, const std::vector<double> & to)
{
  if (from.empty() || from.size() != to.size()) {
    return std::nullopt;
  }
  double largest = 0.0;
  for (size_t i = 0; i < from.size(); ++i) {
    const double move = std::abs(to[i] - from[i]);
    if (!std::isfinite(move)) {
      return std::nullopt;
    }
    largest = std::max(largest, move);
  }
  return largest;
}

std::optional<double> surveyDuration(
  const std::vector<double> & from, const std::vector<double> & to, const SurveyLimits & limits)
{
  const auto move = largestJointMove(from, to);
  if (!move || *move > limits.max_joint_distance || limits.max_joint_speed <= 0.0) {
    return std::nullopt;
  }
  return std::max(limits.min_duration_sec, *move / limits.max_joint_speed);
}

}  // namespace ur30_ibvs
