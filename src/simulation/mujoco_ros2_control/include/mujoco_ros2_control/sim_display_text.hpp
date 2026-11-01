/**
 * Copyright (c) 2026 PAL Robotics S.L.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <cstdio>
#include <string>
#include <utility>

namespace mujoco_ros2_control
{

struct SimDisplayMetrics
{
  double sim_time{ 0.0 };
  double real_time{ 0.0 };
  double time_difference{ 0.0 };
  int constraint_count{ 0 };
  int contact_count{ 0 };
  double cpu_time{ 0.0 };
  double solver_error{ 0.0 };
  int solver_iterations{ 0 };
  double fps{ 0.0 };
  double stack_ratio{ 0.0 };
  int peak_contacts{ 0 };
  int peak_constraints{ 0 };
  bool controller_connected{ false };
  bool running{ false };
  double controller_time_difference{ 0.0 };
};

/**
 * @brief Compose the ICIR-style native-viewer diagnostic overlay.
 *
 * MuJoCo 3.4 allocates contacts and constraints from a dynamic arena.
 * Therefore conbuf and efcbuf show current / observed peak counts instead of
 * the fixed-capacity ratios used by ICIR's MuJoCo 2.x viewer.
 */
inline std::pair<std::string, std::string> compose_sim_display_text(const SimDisplayMetrics& metrics)
{
  char content[768];
  std::snprintf(content, sizeof(content),
                "%.5f\n%.5f\n%+.5f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%d / %d peak\n%d / %d "
                "peak\n%s\n%+.6f",
                metrics.sim_time, metrics.real_time, metrics.time_difference, metrics.constraint_count,
                metrics.contact_count, metrics.cpu_time, metrics.solver_error, metrics.solver_iterations, metrics.fps,
                metrics.stack_ratio, metrics.contact_count, metrics.peak_contacts, metrics.constraint_count,
                metrics.peak_constraints,
                metrics.controller_connected ? (metrics.running ? "CONNECTED / RUN" : "CONNECTED / PAUSE") : "WAITING",
                metrics.controller_time_difference);

  return {
    "Time\nRTime\nt_diff\nSize\nCPU\nSolver\nFPS\nstack\nconbuf\nefcbuf\nController\ntdiff",
    content,
  };
}

}  // namespace mujoco_ros2_control
