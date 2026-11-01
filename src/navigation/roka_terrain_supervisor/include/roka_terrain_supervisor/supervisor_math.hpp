#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace roka_terrain_supervisor
{

inline double speedScaleFromRisk(
  double risk, double warning_ratio = 0.80, double critical_ratio = 0.90,
  double hard_ratio = 1.0)
{
  if (!std::isfinite(risk) || risk >= hard_ratio) {
    return 0.0;
  }
  if (risk <= warning_ratio) {
    return 1.0;
  }
  if (risk <= critical_ratio) {
    const double t = (risk - warning_ratio) / (critical_ratio - warning_ratio);
    return 1.0 - 0.2 * t;
  }
  const double t = (risk - critical_ratio) / (hard_ratio - critical_ratio);
  return std::clamp(0.8 - 0.4 * t, 0.4, 0.8);
}

inline double clampFinite(double value, double limit)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

inline double effectiveTerrainRisk(
  bool terrain_fresh, bool allow_level_fallback, double measured_risk,
  double hard_ratio = 1.0)
{
  if (terrain_fresh) {
    return std::isfinite(measured_risk) ? measured_risk : hard_ratio;
  }
  return allow_level_fallback ? 0.0 : hard_ratio;
}

inline double projectedAbsoluteAngleDeg(
  double angle_deg, double angular_rate_radps, double horizon_sec)
{
  if (!std::isfinite(angle_deg) || !std::isfinite(angular_rate_radps) ||
    !std::isfinite(horizon_sec) || horizon_sec < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double rate_degps = angular_rate_radps * 180.0 / M_PI;
  const double outward_rate = angle_deg * rate_degps > 0.0 ? std::abs(rate_degps) : 0.0;
  return std::abs(angle_deg) + horizon_sec * outward_rate;
}

inline double dynamicRollLoadDeg(
  double relative_roll_deg, double roll_rate_radps, double body_speed_mps,
  double yaw_rate_radps, double horizon_sec)
{
  const double projected = projectedAbsoluteAngleDeg(
    relative_roll_deg, roll_rate_radps, horizon_sec);
  if (!std::isfinite(projected) || !std::isfinite(body_speed_mps) ||
    !std::isfinite(yaw_rate_radps))
  {
    return std::numeric_limits<double>::infinity();
  }
  constexpr double gravity = 9.80665;
  const double equivalent_bank = std::atan2(
    std::abs(body_speed_mps * yaw_rate_radps), gravity) * 180.0 / M_PI;
  return projected + equivalent_bank;
}

inline double attitudeScale(
  double load_deg, double warning_deg, double critical_deg, double hard_deg)
{
  if (!std::isfinite(load_deg) || warning_deg <= 0.0 ||
    !(warning_deg < critical_deg && critical_deg < hard_deg))
  {
    return 0.0;
  }
  if (load_deg <= warning_deg) {
    return 1.0;
  }
  if (load_deg <= critical_deg) {
    const double t = (load_deg - warning_deg) / (critical_deg - warning_deg);
    return 1.0 - 0.25 * t;
  }
  if (load_deg >= hard_deg) {
    return 0.0;
  }
  const double t = (load_deg - critical_deg) / (hard_deg - critical_deg);
  return std::clamp(0.75 - 0.45 * t, 0.30, 0.75);
}

inline double slewLimited(
  double previous, double target, double acceleration_limit,
  double deceleration_limit, double dt)
{
  if (!std::isfinite(previous) || !std::isfinite(target) || !std::isfinite(dt) ||
    acceleration_limit <= 0.0 || deceleration_limit <= 0.0 || dt <= 0.0)
  {
    return 0.0;
  }
  const bool accelerating = std::abs(target) > std::abs(previous) && previous * target >= 0.0;
  const double limit = accelerating ? acceleration_limit : deceleration_limit;
  return previous + std::clamp(target - previous, -limit * dt, limit * dt);
}

}  // namespace roka_terrain_supervisor
