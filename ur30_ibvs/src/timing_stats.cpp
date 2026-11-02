#include "ur30_ibvs/timing_stats.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ur30_ibvs
{

void TimingStats::add(double duration) {samples_.push_back(duration);}

std::optional<TimingSummary> TimingStats::takeSummary()
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  std::sort(samples_.begin(), samples_.end());
  TimingSummary s;
  s.count = samples_.size();
  s.mean = std::accumulate(samples_.begin(), samples_.end(), 0.0) / static_cast<double>(s.count);
  // Nearest rank: the smallest sample with at least 99% of the samples at or below it.
  const auto rank = static_cast<size_t>(std::ceil(0.99 * static_cast<double>(s.count)));
  s.p99 = samples_[rank - 1];
  s.max = samples_.back();
  samples_.clear();
  return s;
}

}  // namespace ur30_ibvs
