#pragma once

#include <algorithm>
#include <cmath>

namespace roka_terrain_mapping
{

struct TraversabilityInput
{
  double slope_deg{0.0};
  double roughness_m{0.0};
  double step_m{0.0};
  double variance_m2{0.0};
  double confidence{1.0};
};

struct TraversabilityLimits
{
  double slope_deg{35.0};
  double roughness_m{0.12};
  double step_m{0.20};
  double variance_m2{0.010};
};

inline double normalizedRisk(const TraversabilityInput & in, const TraversabilityLimits & limits)
{
  const double slope = std::abs(in.slope_deg) / std::max(limits.slope_deg, 1e-6);
  const double roughness = in.roughness_m / std::max(limits.roughness_m, 1e-6);
  const double step = in.step_m / std::max(limits.step_m, 1e-6);
  const double variance = in.variance_m2 / std::max(limits.variance_m2, 1e-6);
  const double geometry = std::max({slope, roughness, step, variance});
  // Confidence is a soft penalty. A geometrically safe cell from one sparse
  // VLP-16 return must not immediately consume 67% of the risk budget.
  const double confidence_penalty = 0.30 * (1.0 - std::clamp(in.confidence, 0.0, 1.0));
  return std::clamp(std::max(geometry, confidence_penalty), 0.0, 2.0);
}

inline double traversabilityFromRisk(double risk)
{
  return std::clamp(1.0 - risk, 0.0, 1.0);
}

}  // namespace roka_terrain_mapping
