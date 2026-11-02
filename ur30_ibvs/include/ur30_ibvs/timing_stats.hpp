#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace ur30_ibvs
{

struct TimingSummary
{
  size_t count = 0;
  double mean = 0.0;
  double p99 = 0.0;
  double max = 0.0;
};

// Collects durations (any unit) between reports. No ROS dependency, so it is easy to test.
class TimingStats
{
public:
  void add(double duration);
  // The summary of what was added since the last call; empty if nothing was.
  std::optional<TimingSummary> takeSummary();

private:
  std::vector<double> samples_;
};

}  // namespace ur30_ibvs
